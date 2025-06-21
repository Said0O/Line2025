# Este archivo une la logica de deteccion de recuadros verdes, analisis de linea negra
# y comportamiento del robot controlado por Raspberry Pi. Se incluye un servidor Flask
# para visualizar en tiempo real el video procesado.

import sys
sys.path.append('/usr/lib/python3/dist-packages')
from flask import Flask, Response
import cv2
import time
import RPi.GPIO as GPIO
import numpy as np
from picamera2 import Picamera2

# === INICIALIZACION ===
app = Flask(__name__)
GPIO.setwarnings(False)

# Pines del motor
ENA = 21
IN1 = 20
IN2 = 16
IN3 = 13
IN4 = 6
ENB = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)

# === FUNCIONES DE MOVIMIENTO ===
def avanzar(vel=100):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(vel)
    pwmB.ChangeDutyCycle(vel)

def girar_derecha(vel_izq=100, vel_der=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(vel_izq)
    pwmB.ChangeDutyCycle(vel_der)

def girar_izquierda(vel_izq=50, vel_der=100):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(vel_izq)
    pwmB.ChangeDutyCycle(vel_der)

def detener():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

# === INICIALIZAR CAMARA ===
picam2 = Picamera2()
picam2.start()

# === CALCULO DE CENTROIDES PARA LA LINEA ===
def girar_sobre_eje_derecha(vel=100):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(vel)
    pwmB.ChangeDutyCycle(vel)

def girar_sobre_eje_izquierda(vel=100):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(vel)
    pwmB.ChangeDutyCycle(vel)

def calcular_centroides(mask, num_segmentos=3):
    alto = mask.shape[0]
    step = alto // (num_segmentos + 1)
    centroides = []
    for i in range(1, num_segmentos + 1):
        h = alto - i * step
        segment = mask[max(0, h-5):min(h+5, alto), :]
        contornos, _ = cv2.findContours(segment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos:
            c = max(contornos, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = h
                centroides.append((cx, cy))
    return centroides

def calcular_desviacion(centroides, ancho_frame=640):
    if centroides:
        promedio_x = np.mean([c[0] for c in centroides])
        centro_x = ancho_frame / 2
        tolerancia = 20
        if abs(promedio_x - centro_x) < tolerancia:
            return 0
        return (promedio_x - centro_x) / centro_x * 100
    return 0

def girar_sobre_eje_derecha(vel=100):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(vel)
    pwmB.ChangeDutyCycle(vel)

def girar_sobre_eje_izquierda(vel=100):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(vel)
    pwmB.ChangeDutyCycle(vel)

def generate_frames():
    desviacion_extra = 0
    en_giro_verde = False
    ya_giro_180 = False

    while True:
        time.sleep(0.03)  # limitar carga CPU (~30 fps)
        frame = picam2.capture_array()
        frame = cv2.resize(frame, (480, 360))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
        mask_green = cv2.inRange(hsv, np.array([35, 100, 50]), np.array([85, 255, 255]))

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        frame_vis = frame

        texto_detectado = []
        verde_con_negro_R = False
        verde_con_negro_L = False
        decision = ""
        boxes = {}

        contornos, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        usados = []

        for cnt in sorted(contornos, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            if any(abs(x - ux) < 30 and abs(y - uy) < 30 for ux, uy, _, _ in usados):
                continue
            usados.append((x, y, w_box, h_box))

            x = max(0, x - 15)
            y = max(0, y - 15)
            w_box += 30
            h_box += 30

            cv2.rectangle(frame_vis, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
            cv2.putText(frame_vis, "Verde", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            roi_top_y = max(0, y - 40)
            roi_top = mask_black[roi_top_y:y, x:x + w_box]
            half_w = w_box // 2
            half_h = 20
            z1 = roi_top[0:half_h, 0:half_w]
            z2 = roi_top[0:half_h, half_w:]
            black_z1 = cv2.countNonZero(z1)
            black_z2 = cv2.countNonZero(z2)

            key = "L" if (x + w_box // 2) < w // 2 else "R"
            boxes[key] = (x, roi_top_y)

            if black_z1 >= 1 and black_z2 >= 1:
                texto_detectado.append(f"Z1+Z2 negras sobre {key}")
                if key == "R":
                    verde_con_negro_R = True
                else:
                    verde_con_negro_L = True
            else:
                texto_detectado.append(f"Descartado {key} (Z1:{black_z1}, Z2:{black_z2})")

        # === GIRO 180° ===
        if verde_con_negro_R and verde_con_negro_L:
            decision = "Giro 180°"
            girar_sobre_eje_derecha(100)
            time.sleep(2.2)
            detener()
            time.sleep(0.4)
            continue

        # === AJUSTAR DESVIACIÓN SI DETECTA VERDE ===
        if verde_con_negro_R:
            desviacion_extra += 32
            en_giro_verde = True
        elif verde_con_negro_L:
            desviacion_extra -= 32
            en_giro_verde = True
        else:
            desviacion_extra = 0
            en_giro_verde = False

        # === NUEVA LÍNEA TRAS INTERSECCIÓN ===
        if en_giro_verde:
            zona_arriba = mask_black[0:h // 4, :]
            if cv2.countNonZero(zona_arriba) > 250:
                texto_detectado.append("Nueva línea detectada arriba -> saliendo de giro")
                desviacion_extra = 0
                en_giro_verde = False

        centroides = calcular_centroides(mask_black)
        desviacion = calcular_desviacion(centroides, ancho_frame=w)
        desviacion_total = desviacion + desviacion_extra

        if -10 <= desviacion_total <= 10:
            avanzar(100)
            decision = "Avanzar recto"
        elif desviacion_total > 10:
            girar_derecha(100, 50)
            decision = "Girar derecha"
        elif desviacion_total < -10:
            girar_izquierda(50, 100)
            decision = "Girar izquierda"
        else:
            detener()
            decision = "Detener"

        debug = [
            f"Desviación real: {desviacion:.2f}",
            f"Desviación extra: {desviacion_extra}",
            f"Total: {desviacion_total:.2f}",
            f"Decisión: {decision}"
        ] + texto_detectado[:4]

        for i, txt in enumerate(debug):
            cv2.putText(frame_vis, txt, (10, 25 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        _, buffer = cv2.imencode('.jpg', frame_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>Streaming del Robot</h1><img src='/video_feed' width='640'>"

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        detener()
        GPIO.cleanup()
