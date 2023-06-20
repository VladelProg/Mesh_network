#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include "heltec.h"

const char * ssid = "Redmi";
const char * password = "";

#define BAND          915E6  //you can set band here directly,e.g. 868E6,915E6
#define BUFFER_SIZE   32 // Define the payload size here

String server = "Redmi";
String eventName = "Vladel";
String Key = "iBdAIIAGz5VVowRemCDE8VII41f0vFOObVa9-60fPJD";


int value1;
int value2;
int value3;

bool bOn = false;
uint16_t rxLen = 0;
char dbgBuf[128];
char rxpacket[BUFFER_SIZE] = {0};
int accX,accY,accZ, tempRaw;
int dataTosend = 0;
int8_t temp = 0;
String RawTemperature(){
  int8_t temp = ((float)tempRaw + 12412.0) / 340.0;
  return String(temp);
}
void setup() {
  Serial.begin(115200);
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, 
      true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.LoRa.setPreambleLength(8);
  Heltec.LoRa.setCodingRate4(5);
  Heltec.LoRa.setSpreadingFactor(7);
  Heltec.LoRa.setSignalBandwidth(125E3);
  //Heltec.LoRa.disableInvertIQ();
  
  //dht.begin();
  bool status;
  LoRa.onReceive(onReceive);
  // put the radio into receive mode
  LoRa.receive();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected !!!");
  
  if( rxLen > 0 )
  {
    bOn = !bOn;
    digitalWrite(LED, bOn ? LOW : HIGH);
    Serial.println(dbgBuf);
    rxLen = 0;

    sscanf(rxpacket, "%d.%d", &dataTosend, &tempRaw);
    temp = ((float)tempRaw + 12412.0) / 340.0;
    Serial.println("Vibration: " + dataTosend );
    Serial.println(" Temperature: " + temp);
    Heltec.display->clear();
    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->drawString(0, rxpacket[5] == '1' ? 0 : 16, rxpacket);
    //Heltec.display->drawString(0, 32, showBuf);
    Heltec.display->display();
  }
  sendDataToSheet();
}

void sendDataToSheet(void)
{
  String url = server + "/trigger/" + eventName + "/with/key/" + IFTTT_Key + "?value1=" + String((int)dataTosend) + "&value2="+String((int)temp);  
  Serial.println(url);
  //Start to send data to IFTTT
  HTTPClient http;
  Serial.print("[HTTP] begin...\n");
  http.begin(url); //HTTP

  Serial.print("[HTTP] GET...\n");
  // start connection and send HTTP header
  int httpCode = http.GET();
  // httpCode will be negative on error
  if(httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    // file found at server
    if(httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();

}

void loop() {

}
void onReceive(int packetSize)
{
  int i;

  // read packet
  for(i = 0; i < packetSize; i++) rxpacket[i] = (char)LoRa.read();

  rxpacket[packetSize] = 0;
  rxLen = sprintf(dbgBuf, "Rcv: %s with RSSI %d", rxpacket, LoRa.packetRssi());
}
