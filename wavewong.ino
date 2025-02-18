#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <ArduinoJson.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

#define MSG_BUFFER_SIZE (50)
#define STASSID "wangcw"
#define STAPSK "12231223"

#define MAX17048_I2C_ADDRESS 0x36
const char *clientID = "8bc56ebe";    // Client ID
char sub[] = "Sub/390/222/8bc56ebe";  // Sub Topic
char pub[] = "Pub/390/222/8bc56ebe";  // Pub Topic
const char *mqtt_server = "mqtt.waveshare.cloud";

StaticJsonDocument<400> sendJson;
StaticJsonDocument<400> readJson;
unsigned long lastUpdateTime = 0;
const char *ssid = STASSID;
const char *password = STAPSK;
char msg[MSG_BUFFER_SIZE];
WiFiClient espClient;
PubSubClient client(espClient);

const unsigned long updateInterval = 5000;

static const int RXPin = 17, TXPin = 18;
static const uint32_t GPSBaud = 115200;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);
String rev;
void SentSerial(const char *p_char) {
  for (int i = 0; i < strlen(p_char); i++) {
    ss.write(p_char[i]);
    delay(10);
  }

  ss.write('\r');
  delay(10);
  ss.write('\n');
  delay(10);
}

bool SentMessage(const char *p_char, unsigned long timeout = 2000) {
  char r_buf[200];

  SentSerial(p_char);

  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (ss.available()) {
      rev = ss.readString();
      char *p_response = strstr(rev.c_str(), "\r\n");
      if (p_response != NULL) {
        p_response += 2;
        if (strstr(p_response, "OK") != NULL) {
          Serial.println("Got OK!");
          return true;
        }
      }
    }
  }
  Serial.println("Timeout!");
  return false;
}

void setup() {
  Wire.begin(3, 2);
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  Serial.begin(9600);
  ss.begin(GPSBaud);

  bool res = false;
  while (!res) {
    res = SentMessage("AT");
    delay(1000);
  }
  res = false;
  SentSerial("AT+CGNSSPWR=1");
  delay(10000);
  SentSerial("AT+CGNSSTST=1");
  delay(500);
  SentSerial("AT+CGNSSPORTSWITCH=0,1");

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop() {
  Wire.beginTransmission(MAX17048_I2C_ADDRESS);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.requestFrom(MAX17048_I2C_ADDRESS, 2);
  uint16_t soc = (Wire.read() << 8) | Wire.read();

  if (soc > 65535) {
    soc = 65535;
  }

  float batteryLevel = (float)soc / 65535.0 * 5;

  sendJson["data"]["batteryLevel"] = batteryLevel;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT,
      LONDON_LON)
    / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT,
      LONDON_LON);
  sendJson["data"]["Latitude"] = gps.location.lat();
  sendJson["data"]["Longitude"] = gps.location.lng();
  sendJsonData();
  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

void callback(char *topic, byte *payload, unsigned int length) {
  String inputString;
  for (int i = 0; i < length; i++) {
    inputString += (char)payload[i];
  }
  Serial.println(inputString);
  int jsonBeginAt = inputString.indexOf("{");
  int jsonEndAt = inputString.lastIndexOf("}");
  if (jsonBeginAt != -1 && jsonEndAt != -1) {
    inputString = inputString.substring(jsonBeginAt, jsonEndAt + 1);
    deserializeJson(readJson, inputString);
  }
}
void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(clientID)) {
      client.subscribe(sub);
    } else {
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void sendJsonData() {
  // sendJson["data"]["Latitude"] = ;
  // sendJson["data"]["Longitude"] = ;

  String pubres;
  serializeJson(sendJson, pubres);
  int str_len = pubres.length() + 1;
  char char_array[str_len];
  pubres.toCharArray(char_array, str_len);
  client.publish(pub, char_array);
}
