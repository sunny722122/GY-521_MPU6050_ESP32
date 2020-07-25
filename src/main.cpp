// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
#include <Arduino.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

#include "TFT_eSPI.h"     // ESP32 Hardware-specific library

#define PHOTOS_pin 36
//#define led1_pin 25
//#define led2_pin 26
#define Piezo_pin 27
#define TMP36_pin 34
#define Poten_pin 35
#define LED_ONBOARD_PIN   2
#define LED1_PIN   25
#define BTN1_PIN   16
#define LED2_PIN   26
#define BTN2_PIN   17


TFT_eSPI tft = TFT_eSPI();

uint16_t bg = TFT_BLUE;
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

int value;
int vtmp36;
int valuep;
//come from arduino tutorial from here
int length = 15; // the number of notes
char notes[] = "ccggaagffeeddc "; // a space represents a rest
int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };
int tempo = 300;

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(Piezo_pin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(Piezo_pin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };
  
  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}
//come from arduino tutorial end here
//https://m1cr0lab-esp32.github.io/remote-control-with-websocket/button-setup/
const uint8_t DEBOUNCE_DELAY = 10; // in milliseconds

// LED
struct Led {
    uint8_t pin;
    bool    on;

    void update() {
        digitalWrite(pin, on ? HIGH : LOW);
    }
};

// Button
struct Button {
    uint8_t  pin;
    bool     lastReading;
    uint32_t lastDebounceTime;
    uint16_t state;

    bool pressed()                { return state == 1; }
    bool released()               { return state == 0xffff; }
    bool held(uint16_t count = 0) { return state > 1 + count && state < 0xffff; }

    void read() {
        bool reading = digitalRead(pin);

        // if the logic level has changed since the last reading
        // reset lastDebounceTime to now
        if (reading != lastReading) {
            lastDebounceTime = millis();
        }

        // after out of the bouncing phase
        // the actual status of the button is determined
        if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
            // the pin is pulled up when not pressed
            bool pressed = reading == LOW;
            if (pressed) {
                     if (state  < 0xfffe) state++;
                else if (state == 0xfffe) state = 2;
            } else if (state) {
                state = state == 0xffff ? 0 : 0xffff;
            }
        }
        lastReading = reading;
    }
};

//https://m1cr0lab-esp32.github.io/remote-control-with-websocket/button-setup/
// Global Variables
Led    onboard_led = { LED_ONBOARD_PIN, false };
Led    led1        = { LED1_PIN, false };
Button button1      = { BTN1_PIN, HIGH, 0, 0 };
Led    led2        = { LED2_PIN, false };
Button button2      = { BTN2_PIN, HIGH, 0, 0 };

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  tft.begin();

  tft.setRotation(3);
  tft.setTextColor(fg, bg);
  
  tft.fillScreen(bg);
  
  tft.setCursor(0, 0);
  tft.println("Hello!");
  tft.println("Starting GY-521...");
  delay(5000);
  tft.fillScreen(bg);
  pinMode(PHOTOS_pin,INPUT);
  pinMode(onboard_led.pin,  OUTPUT);
  pinMode(led1.pin,         OUTPUT);
  pinMode(button1.pin,      INPUT);
  pinMode(led2.pin,         OUTPUT);
  pinMode(button2.pin,      INPUT);

  pinMode(TMP36_pin,INPUT);
  pinMode(Poten_pin,INPUT);
  pinMode(Piezo_pin,OUTPUT);
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
  
  value=analogRead(PHOTOS_pin);
  vtmp36=analogRead(TMP36_pin);
  valuep=analogRead(Poten_pin);
  dacWrite(led1.pin,valuep);
  button1.read();
  if (button1.pressed()) {
      ;//led1.on = !led1.on;
  }
  button2.read();
  // if (button2.held())     led2.on = true;
  // else if (button2.released()) led2.on = false;
  // led1.update();
  // led2.update();
  //onboard_led.on = millis() % 1000 < 500;
  //onboard_led.update();
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

  tft.fillRect(5, 170, 200, 30, bg);
  tft.setCursor(5, 170);
  tft.print("Photo in= ");
  tft.print(convert_int16_to_str(value));
  tft.print("| tmp36= ");
  tft.print(vtmp36/340.00+36.53);
  tft.print(" °C");
  tft.print("| analog in= ");
  tft.print(convert_int16_to_str(valuep));
  tft.println();

  for (int i = 0; i < length; i++) 
  {
    if (notes[i] == ' ') {
      delay(beats[i] * tempo); // rest
    } else {
      playNote(notes[i], beats[i] * tempo);
    }
    // pause between notes
    delay(tempo / 2); 
  }

  delay(2000);
}