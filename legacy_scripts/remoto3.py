import pigpio, serial, threading, time, cv2, numpy as np
import pyrealsense2 as rs
from flask import Flask, Response

# Configuración Hardware
ESC, SERVO = 18, 13
L_BLANCO, L_ROJO, G_IZQ, G_DER = 26, 27, 22, 23
NEUTRO, PASO_V, PASO_D = 1500, 80, 150
v_act, d_act = NEUTRO, NEUTRO

pi = pigpio.pi()
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
for p in [L_BLANCO, L_ROJO, G_IZQ, G_DER]: pi.set_mode(p, pigpio.OUTPUT)

# Cámara RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

app = Flask(__name__)

def gen_frames():
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue
        frame = np.asanyarray(color_frame.get_data())
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

def leer_serial():
    global v_act, d_act
    while True:
        if ser.in_waiting > 0:
            cmd = ser.read(1).decode('utf-8').lower()
            if cmd == 'w': v_act = min(2000, v_act + PASO_V)
            elif cmd == 's': v_act = max(1400, v_act - PASO_V)
            elif cmd == 'a': d_act = max(1200, d_act - PASO_D)
            elif cmd == 'd': d_act = min(1800, d_act + PASO_D)
            elif cmd == 'c': d_act = NEUTRO
            pi.set_servo_pulsewidth(ESC, v_act)
            pi.set_servo_pulsewidth(SERVO, d_act)

threading.Thread(target=leer_serial, daemon=True).start()

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index(): return '<html><body style="background:#000;text-align:center;"><img src="/video_feed" width="80%"></body></html>'

if __name__ == '__main__':
    try: app.run(host='0.0.0.0', port=5000, threaded=True)
    finally: pipeline.stop(); pi.stop()
