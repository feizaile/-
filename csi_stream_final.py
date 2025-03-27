from picamera2 import Picamera2
from flask import Flask, Response, render_template_string
import cv2
import time
import numpy as np

app = Flask(__name__)
picam2 = Picamera2()

# 参数配置
LUX_THRESHOLD = 30
NIGHT_MATRIX = np.array([[1.8, -0.5, 0.3], [-0.2, 1.6, -0.4], [0.1, -0.3, 1.7]])
DAY_MATRIX = np.array([[1.25, -0.18, 0.08], [-0.05, 1.12, -0.07], [0.03, -0.15, 1.25]])

def setup_camera():
    config = picam2.create_video_configuration(
        main={"size": (1280, 720), "format": "YUV420"},
        controls={"AwbEnable": False}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

def get_brightness(frame):
    y_channel = frame[:720, :]
    return np.mean(y_channel)

@app.route('/api/status')
def get_status():
    frame = picam2.capture_array("main")
    brightness = int(get_brightness(frame))
    return {
        "brightness": brightness,
        "light_status": "激活" if brightness < LUX_THRESHOLD else "关闭"
    }

@app.route('/')
def control_panel():
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>智能补光系统</title>
        <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('brightness').innerText = data.brightness + '%';
                    document.getElementById('light-status').innerText = data.light_status;
                });
            setTimeout(updateStatus, 1000);
        }
        window.onload = updateStatus;
        </script>
    </head>
    <body>
        <h1>智能补光系统</h1>
        <p>环境亮度: <span id="brightness">--</span></p>
        <p>补光状态: <span id="light-status">--</span></p>
        <img src="/video" width="640" height="360">
    </body>
    </html>
    ''')

@app.route('/video')
def video_feed():
    def generate():
        while True:
            frame = picam2.capture_array("main")
            brightness = get_brightness(frame)
            matrix = NIGHT_MATRIX if brightness < LUX_THRESHOLD else DAY_MATRIX
            rgb = cv2.cvtColor(frame, cv2.COLOR_YUV420p2RGB)
            corrected = cv2.transform(rgb, matrix)
            _, buffer = cv2.imencode('.jpg', corrected, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        setup_camera()
        app.run(host='192.168.31.54', port=8000, threaded=True)
    finally:
        picam2.close()