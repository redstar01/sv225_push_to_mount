/*
 * Описание:
 *  Прошивка esp32 для передачи значений энкодеров (MT6701) астро-монтировок для работы в Sky Safari
 *
 * Автор: Красноперов Павел
 *   https://t.me/redstar01
 *
 * Документация, новые версии программы:
 *   https://github.com/redstar01/sv225_push_to_mount
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <Wire.h>

const char *ssid = "redstar01";
const char *password = "1234567890";

WiFiServer server(80);
WiFiClient client;

void setup()
{
  delay(1000);

  WiFi.softAP(ssid, password);
  server.begin();
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin(18,19);
  Wire1.setClock(400000);
}

void loop() {
  int azAngle = readAngle(Wire1);
  int altAngle = readAngle(Wire);

  String logMsg;
  logMsg += "азимут: ";
  logMsg += azAngle;
  logMsg += " ### ";
  logMsg += "высота: ";
  logMsg += altAngle;
  logMsg += "\n";

  Serial.print(logMsg);

  client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        int c = client.read();
        if (c == 81) {
         printEncoderValue(azAngle);
         client.print("\t");
         printEncoderValue(altAngle);
         client.print("\r");
        }
      }
    }
  }

  delay(200);
}

int readAngle(TwoWire& wire)
{
    uint8_t data[2];
    wire.beginTransmission(0b0000110);
    wire.write(0x03);
    wire.endTransmission(false);
    wire.requestFrom((int)0b0000110, 2);
    if (wire.available() < 2)
    {
        return -1;
    }
    int angle_h = wire.read();
    int angle_l = wire.read();

    return (angle_h << 6) | (angle_l >> 2);
}

void printEncoderValue(long val)
{
  if (val < 10)
    client.print("0000");
  else if (val < 100)
    client.print("000");
  else if (val < 1000)
    client.print("00");
  else if (val < 10000)
    client.print("0");

  client.print(val);
}
