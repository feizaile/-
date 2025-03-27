import subprocess
import signal
import sys
import time

processes = []

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n正在关闭所有子进程...")
    for p in processes:
        p.terminate()
    sys.exit(0)

def run_script(script_name):
    """启动子进程"""
    return subprocess.Popen([sys.executable, script_name])

if __name__ == '__main__':
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 启动子进程
    scripts = ['ps4_final_control.py', 'csi_stream_final.py']
    for script in scripts:
        p = run_script(script)
        processes.append(p)
        print(f"已启动 {script} (PID: {p.pid})")
    
    # 监控进程状态
    try:
        while True:
            for p in processes.copy():
                retcode = p.poll()
                if retcode is not None:
                    print(f"进程 {p.pid} 意外退出，代码: {retcode}")
                    processes.remove(p)
            time.sleep(1)
    except KeyboardInterrupt:
        pass