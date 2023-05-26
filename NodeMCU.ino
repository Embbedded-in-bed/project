/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL6SpNP52z7"
#define BLYNK_TEMPLATE_NAME "Nodemcu"
#define BLYNK_AUTH_TOKEN "LaaOmXLitG_25SG8ZK7RVu4tVHKfSnox"

#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleEsp8266.h>

#define LED_PIN D0
#define FINE_PIN V1
#define WARN_PIN V2
#define DANG_PIN V3
#define WATER_PIN V0
#define TEMP_PIN V4

SoftwareSerial BlynkSerial(D7, D8); // Rx, Tx

String text = "";
String temp = "";
String level = "";
int temp_val = 25;
int level_val = 0;
int water_max_level = 1100;

// Your WiFi credentials.
// Please enter your ssid and password
char ssid[] = "";
char pass[] = "";

void receiveData() {
  // Serial.println(BlynkSerial.available());
  // Serial.println(BlynkSerial.peek());
  text = "";
  while(BlynkSerial.available() > 0) {
    text += (char)BlynkSerial.read();
  }

  if(text != "") {
    if (text.indexOf("0") == 0) { 
      temp = text;
      if (temp.toInt()<100) temp_val = temp.toInt();
      Serial.print("Temperature:");
      writeBlynkTemp();
    } else { 
        level = text;
        if(level.toInt() < water_max_level) {
          level_val = water_max_level - level.toInt();
        }
      Serial.print("Water Level:");
      writeBlynkLevel();
    }
    Serial.println(text);
  }
}

void writeState() {
  if (level_val > 900 || (level_val > 700 && temp_val < 20)) { 
    dangerous(); 
  }
  else if (level_val > 700) {  
    warning(); 
  }
  else { 
    fine(); 
  }
}

void writeBlynkTemp(){
  Blynk.virtualWrite(TEMP_PIN, temp_val);
}

void writeBlynkLevel(){
  Blynk.virtualWrite(WATER_PIN, level_val);
}

void fine() {
  Blynk.virtualWrite(FINE_PIN, 1);
  Blynk.virtualWrite(WARN_PIN, 0);
  Blynk.virtualWrite(DANG_PIN, 0);
}

void warning() {
  Blynk.virtualWrite(FINE_PIN, 0);
  Blynk.virtualWrite(WARN_PIN, 1);
  Blynk.virtualWrite(DANG_PIN, 0);
}

void dangerous() {
  Blynk.virtualWrite(FINE_PIN, 0);
  Blynk.virtualWrite(WARN_PIN, 0);
  Blynk.virtualWrite(DANG_PIN, 1);
}

void setup() {
  Serial.begin(115200);
  BlynkSerial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("ready");
  pinMode(LED_PIN, 0);
  fine();
  writeBlynkTemp();
  writeBlynkLevel();
}

void loop()
{
  Blynk.run();
  receiveData();
  writeState();
}