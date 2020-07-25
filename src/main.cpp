// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
#include <Arduino.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

#include "TFT_eSPI.h"     // ESP32 Hardware-specific library
#include "WiFi.h"
//#include <ESPAsyncWebServer.h>
#include <WebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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
#define HTTP_PORT 80

#define SEALEVELPRESSURE_HPA (1013.25)
// WiFi credentials
const char *WIFI_SSID = "TELUS1005";
const char *WIFI_PASS = "nfnb33pgv2";

TFT_eSPI tft = TFT_eSPI();

uint16_t bg = TFT_BLUE;
uint16_t fg = TFT_WHITE ;

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

Adafruit_BME280 bme;

float temperature, humidity, pressure, altitude;
WebServer server(80);   
void handle_OnConnect();  
void handle_NotFound();  
String SendHTML(float temperature,float humidity,float pressure,float altitude);      
 

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

// Global Variables
Led    onboard_led = { LED_ONBOARD_PIN, false };
Led    led1        = { LED1_PIN, false };
Button button1      = { BTN1_PIN, HIGH, 0, 0 };
Led    led2        = { LED2_PIN, false };
Button button2      = { BTN2_PIN, HIGH, 0, 0 };
// ----------------------------------------------------------------------------
// SPIFFS initialization
// ----------------------------------------------------------------------------

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("Cannot mount SPIFFS volume...");
    while (1) {
        onboard_led.on = millis() % 200 < 50;
        onboard_led.update();
    }
  }
}
// ----------------------------------------------------------------------------
// Connecting to the WiFi network
// ----------------------------------------------------------------------------

void initWiFi() {
  //connect to your local wi-fi network
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

String processor(const String &var) {
    return String(var == "STATE" && led1.on ? "on" : "off");
}

//https://m1cr0lab-esp32.github.io/remote-control-with-websocket/button-setup/

void setup() {
  Serial.begin(9600);

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
  bme.begin(0x76); 
  //initSPIFFS();
  initWiFi();

  
}
void loop() {
  server.handleClient();
  handle_OnConnect();


  value=analogRead(PHOTOS_pin);
  vtmp36=analogRead(TMP36_pin);
  valuep=analogRead(Poten_pin);
  dacWrite(led1.pin,valuep);
  
  Serial.print("tmp = "); 
  Serial.print(temperature);
  Serial.print(" | Humidity = "); Serial.print(humidity);
  Serial.print(" | pressure = "); Serial.print(pressure);
  Serial.print(" | altitude = "); Serial.print(altitude);
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
  tft.print("Temp= ");
  tft.print(temperature);
  tft.print("| Humidity= ");
  tft.print(humidity);
  tft.print("| Pressure= ");
  tft.print(pressure);
  tft.print("| Altitude= ");
  tft.print(altitude);
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


void handle_OnConnect() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  server.send(200, "text/html", SendHTML(temperature,humidity,pressure,altitude)); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(float temperature,float humidity,float pressure,float altitude){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>ESP32 Weather Station</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  ptr +="p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<div id=\"webpage\">\n";
  ptr +="<h1>ESP32 Weather Station</h1>\n";
  ptr +="<p>Temperature: ";
  ptr +=temperature;
  ptr +="&deg;C</p>";
  ptr +="<p>Humidity: ";
  ptr +=humidity;
  ptr +="%</p>";
  ptr +="<p>Pressure: ";
  ptr +=pressure;
  ptr +="hPa</p>";
  ptr +="<p>Altitude: ";
  ptr +=altitude;
  ptr +="m</p>";
  ptr +="</div>\n";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}