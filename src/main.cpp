// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
#include <Arduino.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.
//#include "sensor_readings.h"
#include "TFT_eSPI.h"     // ESP32 Hardware-specific library
//#include "settings.h"    // The order is important!
// bme is global to this file only
//Adafruit_BME280 bme;

TFT_eSPI tft = TFT_eSPI();

uint16_t bg = TFT_RED;
uint16_t fg = TFT_WHITE ;

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //digitalWrite(15, HIGH);
  tft.begin();

  tft.setRotation(3);
  tft.setTextColor(fg, bg);
  
  tft.fillScreen(bg);
  
  tft.setCursor(0, 0);
  tft.println("Hello!");
  tft.println("Starting GY-521...");
  delay(5000);
  tft.fillScreen(bg);
}
void loop() {
  

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  tft.setCursor(50, 50);

  tft.println(millis());
  //refresh_readings(&bme, &tft);  
  //bg = TFT_BLACK;
  //fg = TFT_WHITE;

  tft.setCursor(5, 5);
  tft.setTextColor(fg, bg);
  tft.println("Right now...");

  tft.setTextColor(TFT_YELLOW, bg);
  tft.fillRect(5, 50, 200, 30, bg);
  tft.setCursor(5, 50);
  tft.print("tmp= ");
  tft.print(temperature/340.00+36.53);
  tft.println(" °C");

  tft.fillRect(5, 90, 200, 30, bg);
  tft.setCursor(5, 90);
  tft.print("ax= ");
  tft.print(convert_int16_to_str(accelerometer_x));
  tft.print("| ay= ");
  tft.print(convert_int16_to_str(accelerometer_y));
  tft.print("| az= ");
  tft.print(convert_int16_to_str(accelerometer_z));
  tft.println();

  tft.fillRect(5, 130, 200, 30, bg);
  tft.setCursor(5, 130);
  tft.print("gx= ");
  tft.print(convert_int16_to_str(gyro_x));
  tft.print("| gy= ");
  tft.print(convert_int16_to_str(gyro_y));
  tft.print("| gz= ");
  tft.print(convert_int16_to_str(gyro_z));
  tft.println();

  // tft.fillRect(5, 170, 200, 30, bg);
  // tft.setCursor(5, 170);
  // tft.print("Accelerometer z: ");
  // tft.print(convert_int16_to_str(accelerometer_z)); 
  // tft.println(" mm");
  //delay(2000);
  // delay
  delay(2000);
}