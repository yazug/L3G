#include <Wire.h>
#include <L3GD20.h>

L3GD20 gyro;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  gyro.enableDefault();
}

int16_t x;
int16_t y;
int16_t z;
int8_t temp;

void loop() {

  gyro.readGyro(&x,&y,&z);

  Serial.print("G ");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  gyro.readTemperature(&temp);
  Serial.print("T: ");
  Serial.print(temp);

  delay(100);
}

