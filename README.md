# GY-521_MPU6050_ESP32
Diagram and part of the code referenced from Robbin Law


NODEMCU_32s
Connection:
GY-521.  ESP32
VCC-     3.3V
GND.     GND
SCL      IO22
SDA.     IO21

ILI9341.   ESP32
VCC         5V
GND         GND
TFT_CS      IO15
TFT_RESET   IO2
TFT_DC      IO0
MOSI        IO23
SCK.        IO18
MISO        IO19

Need to change library User_Setup.h file setting
enable ESP32 TFT_CS ... disable default one
and need to change TFT_DC to 0, TFT_RS to 2
