#include <Servo.h>

Servo servoX;  // Servo del eje X (horizontal)
Servo servoY;  // Servo del eje Y (vertical)

int posX = 90;  // Posición inicial en grados
int posY = 90;

void setup() {
  Serial.begin(115200);
  
  // Asignar los pines de los servos
  servoX.attach(9);
  servoY.attach(10);

  // Mover servos a la posición inicial
  servoX.write(posX);
  servoY.write(posY);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();  // Limpiar cualquier carácter adicional

    // Parsear los ángulos recibidos del formato "X,Y"
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      String xString = data.substring(0, commaIndex);
      String yString = data.substring(commaIndex + 1);

      // Convertir a enteros
      posX = xString.toInt();
      posY = yString.toInt();

      // Limitar los ángulos entre 0 y 180 grados
      posX = constrain(posX, 0, 180);
      posY = constrain(posY, 0, 180);

      // Mover los servos a las posiciones especificadas
      servoX.write(posX);
      servoY.write(posY);
    }
  }
}
