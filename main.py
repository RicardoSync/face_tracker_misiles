import cv2
import mediapipe as mp
import serial
import time

# Inicializar Mediapipe
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh()

# Configurar conexión serial con Arduino (cambiar el puerto según sea necesario)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Cambiar '/dev/ttyACM0' según tu puerto
time.sleep(2)  # Espera a que se inicie la conexión

# Función para mapear valores
def map_value(value, min_input, max_input, min_output, max_output):
    return (value - min_input) * (max_output - min_output) / (max_input - min_input) + min_output

# Capturar video desde la webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir la imagen a RGB (Mediapipe requiere RGB)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            # Obtener las coordenadas de la nariz (landmark 1)
            h, w, _ = frame.shape
            x = int(face_landmarks.landmark[1].x * w)
            y = int(face_landmarks.landmark[1].y * h)

            # Dibujar punto en la nariz
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

            # Mapear las coordenadas (0-640 para X, 0-480 para Y) a ángulos de servo (0-180 grados)
            # Invertir los valores para los servos
            servo_x = 180 - map_value(x, 0, w, 0, 180)  # Invertir el eje X
            servo_y = 180 - map_value(y, 0, h, 0, 180)  # Invertir el eje Y

            # Mostrar los valores en la pantalla
            cv2.putText(frame, f"Servo X: {int(servo_x)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Servo Y: {int(servo_y)}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            # Enviar los valores al Arduino a través de Serial
            arduino.write(f'{int(servo_x)},{int(servo_y)}\n'.encode('utf-8'))
            print(f'{int(servo_x)},{int(servo_y)}')

    # Mostrar la imagen con el punto de seguimiento y los valores
    cv2.imshow("Face Tracker", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
arduino.close()
