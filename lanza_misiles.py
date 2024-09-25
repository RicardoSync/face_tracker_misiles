import cv2
import mediapipe as mp
import serial
import time
import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# Inicializar Mediapipe
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh()

# Configurar conexión serial con Arduino (cambiar el puerto según sea necesario)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Cambiar '/dev/ttyACM0' según tu puerto
time.sleep(2)  # Espera a que se inicie la conexión

# Función para mapear valores
def map_value(value, min_input, max_input, min_output, max_output):
    return (value - min_input) * (max_output - min_output) / (max_input - min_input) + min_output

# Crear la ventana principal de CustomTkinter
root = ctk.CTk()
root.geometry("800x600")
root.title("Seguimiento de Rostro y Gráfica en Tiempo Real")

# Crear el frame para contener la gráfica
frame_grafica = ctk.CTkFrame(master=root)
frame_grafica.pack(side="right", fill="both", expand=True)

# Inicializar Matplotlib para la gráfica de X e Y
fig, ax = plt.subplots()
x_data = []
y_data = []
servo_x_values = []
servo_y_values = []

# Configuración de la gráfica
ax.set_xlim(0, 100)  # Limitar el eje X para que muestre los últimos 100 puntos
ax.set_ylim(0, 180)  # El ángulo del servo está entre 0 y 180 grados
line_x, = ax.plot([], [], label="Servo X", color='r')
line_y, = ax.plot([], [], label="Servo Y", color='b')
ax.legend()

# Incrustar la gráfica en el CTkFrame
canvas = FigureCanvasTkAgg(fig, master=frame_grafica)
canvas.get_tk_widget().pack(fill="both", expand=True)

# Función para actualizar la gráfica
def actualizar_grafica():
    line_x.set_data(np.arange(len(servo_x_values)), servo_x_values)
    line_y.set_data(np.arange(len(servo_y_values)), servo_y_values)
    
    ax.set_xlim(max(0, len(servo_x_values) - 100), len(servo_x_values))  # Mantener ventana de los últimos 100 puntos
    canvas.draw()

# Capturar video desde la webcam
cap = cv2.VideoCapture(0)

# Función principal del bucle de video y actualización
def loop():
    ret, frame = cap.read()
    if not ret:
        return

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

            # Mapear las coordenadas (invertir los ejes para los servos)
            servo_x = 180 - map_value(x, 0, w, 0, 180)
            servo_y = 180 - map_value(y, 0, h, 0, 180)

            # Mostrar los valores en la imagen
            cv2.putText(frame, f"Servo X: {int(servo_x)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Servo Y: {int(servo_y)}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            # Enviar los valores al Arduino
            arduino.write(f'{int(servo_x)},{int(servo_y)}\n'.encode('utf-8'))

            # Actualizar los datos para la gráfica
            servo_x_values.append(servo_x)
            servo_y_values.append(servo_y)
            
            # Limitar el tamaño de las listas a 100 puntos para evitar crecimiento excesivo
            if len(servo_x_values) > 100:
                servo_x_values.pop(0)
                servo_y_values.pop(0)

    # Actualizar la gráfica
    actualizar_grafica()

    # Mostrar la imagen en una ventana (opcional)
    cv2.imshow("Face Tracker", frame)

    # Salir si se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        arduino.close()
        root.quit()

    # Llamar a esta función nuevamente
    root.after(10, loop)

# Iniciar el loop
root.after(10, loop)
root.mainloop()
