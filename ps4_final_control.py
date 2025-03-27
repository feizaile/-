import RPi.GPIO as GPIO
import pygame
import time
import sys
import numpy as np
import subprocess

# === 硬件配置 ===
# 舵机配置
SERVO_PIN = 17
# 电机配置
MOTOR_A = {
    'pwm': 18,    # GPIO18 硬件PWM0
    'in1': 22,    # 正转信号
    'in2': 23     # 反转信号
}
MOTOR_B = {
    'pwm': 19,    # GPIO19 硬件PWM1
    'in1': 24,
    'in2': 25
}

# PS4手柄轴配置
AXIS_X = 3        # 舵机方向控制（右摇杆左右）
TRIGGER_R2 = 4    # 右扳机轴
TRIGGER_L2 = 5    # 左扳机轴

# 舵机参数配置
DEAD_ZONE = 0.15        # 基础死区
LIMIT_LOCK_ZONE = 0.98  # 极限锁定区（摇杆推到底的阈值）
SMOOTHING_ACTIVE_ZONE = 0.8  # 平滑生效区（非极限区才启用平滑）
ANGLE_LIMIT = 45        # 舵机角度限制
PWM_OFF_DELAY = 0.2     # 到达目标后关闭PWM的延迟

class MotorController:
    def __init__(self):
        # 关键参数初始化
        self.pwm_freq = 2000    # PWM频率2kHz
        self.deadzone = 0.05    # 5%死区消除摇杆漂移
        
        # 初始化硬件
        self._init_gpio()
        print("电机控制器就绪")

    def _init_gpio(self):
        """初始化GPIO和PWM"""
        # 方向控制引脚
        for m in [MOTOR_A, MOTOR_B]:
            GPIO.setup(m['in1'], GPIO.OUT)
            GPIO.setup(m['in2'], GPIO.OUT)
            GPIO.output(m['in1'], GPIO.LOW)
            GPIO.output(m['in2'], GPIO.LOW)
        
        # PWM配置
        GPIO.setup(MOTOR_A['pwm'], GPIO.OUT)
        GPIO.setup(MOTOR_B['pwm'], GPIO.OUT)
        self.pwm_a = GPIO.PWM(MOTOR_A['pwm'], self.pwm_freq)
        self.pwm_b = GPIO.PWM(MOTOR_B['pwm'], self.pwm_freq)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
    
    def _set_motor_direction(self, motor, direction):
        """设置单个电机方向（优化信号锁存）"""
        # direction: 0=停止, 1=正转, -1=反转
        if direction == 1:
            GPIO.output(motor['in1'], GPIO.HIGH)
            GPIO.output(motor['in2'], GPIO.LOW)
        elif direction == -1:
            GPIO.output(motor['in1'], GPIO.LOW)
            GPIO.output(motor['in2'], GPIO.HIGH)
        else:
            GPIO.output(motor['in1'], GPIO.LOW)
            GPIO.output(motor['in2'], GPIO.LOW)

    def move(self, speed):
        """主控制函数（speed范围：-1.0~1.0）"""
        if abs(speed) < self.deadzone:
            self.stop()
            return
        
        # 确定方向
        direction = 1 if speed > 0 else -1
        speed = abs(speed)
        
        # 设置双电机方向
        self._set_motor_direction(MOTOR_A, direction)
        self._set_motor_direction(MOTOR_B, direction)
        
        # 计算PWM（指数曲线优化控制）
        pwm = (speed ** 1.5) * 100  # 低速更平顺
        
        self.pwm_a.ChangeDutyCycle(pwm)
        self.pwm_b.ChangeDutyCycle(pwm)
    
    def stop(self):
        """急停功能"""
        self._set_motor_direction(MOTOR_A, 0)
        self._set_motor_direction(MOTOR_B, 0)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

class ServoController:
    def __init__(self, servo_pin):
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz PWM
        self.pwm.start(0)
        
        # 状态跟踪变量
        self.filtered_angle = 90
        self.last_angle = 90
        self.last_pwm_active_time = 0
        
        print("舵机控制器就绪")
        
    def map_angle(self, x):
        """增强型角度映射（极限区直接锁定）"""
        # 摇杆推到底的精确判断
        if x <= -LIMIT_LOCK_ZONE:
            return 90 - ANGLE_LIMIT
        elif x >= LIMIT_LOCK_ZONE:
            return 90 + ANGLE_LIMIT
        # 常规区域处理
        return np.interp(x, [-LIMIT_LOCK_ZONE, LIMIT_LOCK_ZONE], 
                        [90 - ANGLE_LIMIT, 90 + ANGLE_LIMIT])

    def update_servo(self, angle):
        """带硬件保护的舵机控制"""
        # 双重角度限制（软件+硬件）
        angle = np.clip(angle, 90 - ANGLE_LIMIT, 90 + ANGLE_LIMIT)
        # 计算精确占空比（CLS6336HV需微调）
        duty_cycle = 2.5 + (angle / 18)
        # 执行PWM输出
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.last_pwm_active_time = time.time()  # 记录最后活动时间
        return angle
        
    def process_input(self, axis_x):
        """处理输入并更新舵机位置"""
        current_time = time.time()

        # 信号预处理（四舍五入消除微小波动）
        processed_x = round(axis_x, 2)  # 保留2位小数消除噪声
        
        # 直接锁定极限位置
        if abs(processed_x) >= LIMIT_LOCK_ZONE:
            # 极限区禁用平滑，直接设置目标角度
            target_angle = 90 - ANGLE_LIMIT if processed_x < 0 else 90 + ANGLE_LIMIT
            self.filtered_angle = target_angle
        else:
            # 非极限区使用原有逻辑
            target_angle = self.map_angle(processed_x)
            # 仅在有效活动区启用平滑
            if abs(processed_x) < SMOOTHING_ACTIVE_ZONE:
                self.filtered_angle = 0.3 * target_angle + 0.7 * self.filtered_angle

        # 更新舵机（带变化阈值检测）
        if abs(self.filtered_angle - self.last_angle) > 0.1:  # 降低敏感度
            self.last_angle = self.update_servo(self.filtered_angle)
        else:
            # 静止后延迟关闭PWM
            if current_time - self.last_pwm_active_time > PWM_OFF_DELAY:
                self.pwm.ChangeDutyCycle(0)

        # 返回状态信息
        status = "LOCKED" if abs(processed_x) >= LIMIT_LOCK_ZONE else "ACTIVE"
        return status, self.filtered_angle
        
    def cleanup(self):
        """清理舵机资源"""
        self.pwm.stop()

class PS4Controller:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.js = None
        self._connect()
    
    def _connect(self):
        """连接手柄（增加异常处理）"""
        while True:
            try:
                if pygame.joystick.get_count() > 0:
                    self.js = pygame.joystick.Joystick(0)
                    self.js.init()
                    print(f"已连接手柄: {self.js.get_name()}")
                    return
                time.sleep(1)
            except pygame.error as e:
                print(f"连接错误: {str(e)}")
                time.sleep(2)
    
    def get_inputs(self):
        """获取所有需要的输入值"""
        pygame.event.pump()
        
        # 获取舵机控制值（右摇杆左右）
        axis_x = self.js.get_axis(AXIS_X)
        
        # 读取扳机值（范围修正）
        r2 = (self.js.get_axis(TRIGGER_R2) + 1) / 2  # 0.0~1.0
        l2 = (self.js.get_axis(TRIGGER_L2) + 1) / 2
        
        # 合并控制信号（R2正方向，L2负方向）
        speed = r2 - l2  # 范围：-1.0~1.0
        
        return axis_x, speed

def main():
    # 硬件初始化命令（每次启动后执行）
    subprocess.run(["sudo", "raspi-gpio", "set", "18", "a5"])  # PWM0
    subprocess.run(["sudo", "raspi-gpio", "set", "19", "a5"])  # PWM1
    
    # 设置GPIO模式
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 创建控制器
    servo = ServoController(SERVO_PIN)
    motor = MotorController()
    ps4 = PS4Controller()
    
    try:
        print("控制模式：右摇杆左右=方向 | R2=前进 | L2=后退")
        while True:
            # 获取手柄输入
            axis_x, speed = ps4.get_inputs()
            
            # 处理舵机控制
            servo_status, servo_angle = servo.process_input(axis_x)
            
            # 处理电机控制
            if abs(speed) > motor.deadzone:
                motor.move(speed)
                motor_status = f"MOVE: {speed:.2f}"
            else:
                motor.stop()
                motor_status = "STOP"
            
            # 显示状态
            sys.stdout.write(f"\r舵机: {servo_status:6} | 角度: {servo_angle:6.1f}° | 电机: {motor_status:10}")
            sys.stdout.flush()
            
            time.sleep(0.02)
    
    except KeyboardInterrupt:
        pass
    finally:
        servo.cleanup()
        motor.cleanup()
        GPIO.cleanup()
        print("\n系统已安全关闭")

if __name__ == "__main__":
    main()