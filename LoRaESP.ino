#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"
#include<Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//array of frequencies valid for the application to change
long int frequencies[6] = {915000000, 915125000, 868000000, 868125000, 433000000, 433125000};
//controls the current frequency index in the array
int indexFreq = 0;

// WiFi network name and password:
const char * networkName = "XATA";
const char * networkPswd = "password";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.1.255";
const int udpPort = 3333;

//Are we currently connected to WiFi?
boolean connectedWiFi = false;

//The udp library class
WiFiUDP udp;

//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16

SSD1306  display(0x3c, 4, 15);

// WIFI_LoRa_32 ports

// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

#define SS      18
#define RST     14
#define DI0     26
#define BAND    433E6  //915E6 

// int counter = 0;
String RxRssi = "0";
String TxRssi = "0";
//int RxCount = 0;
int TxCount = 0;
int timerCount = 0;

void setup() {
  pinMode(25,OUTPUT); //Send success, LED will bright 1 second
  
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH);
  
  Serial.begin(115200);
  while (!Serial); //If just the the basic function, must connect to a computer
 
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender&Reciever");
  display.display();
  
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender&reciver");
  
  if (!LoRa.begin(BAND)) {
    // Serial.println("Starting LoRa failed!");
      display.drawString(5,25,"Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
  display.drawString(5,25,"LoRa Initializing OK!");
  //show working Band
  display.drawString(5, 35, String(frequencies[indexFreq]));
  display.display();
  delay(5000);
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop() {
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  Serial.print("packetSize= ");
  Serial.println(LoRa.parsePacket());
  
  if ((packetSize) || (TxCount == 0)) {
    while (LoRa.available()) {
      TxRssi = LoRa.readString();
    }   
       // send packet
      LoRa.beginPacket();
      LoRa.print((String)LoRa.packetRssi());
      LoRa.endPacket();
      Serial.print("(String)LoRa.packetRssi()= ");
      Serial.println((String)LoRa.packetRssi());
      TxCount++;
      timerCount=0;
    }
      Serial.print("TxRssi= ");
      Serial.println(TxRssi);
      
  display.clear();
  display.setFont(ArialMT_Plain_16); 
  display.drawString(3, 5, "RxCount:");
  display.drawString(75, 5, (String)TxCount);
  
  display.drawString(3, 30, "TX RSSI:");  
  display.drawString(75, 30, TxRssi);

  if (timerCount < 500) { 
    RxRssi = (String)LoRa.packetRssi();
  } else {
    RxRssi = "lost";
  }
  display.drawString(3, 45, "RX RSSI:");
  display.drawString(75, 45, RxRssi);
  display.display();
      Serial.print("RxRssi= ");
      Serial.println(RxRssi);

  if (timerCount > 500) {
       // send packet
        LoRa.beginPacket();
        LoRa.print("lost");
        LoRa.endPacket();
        TxCount++;
        timerCount=0;
  }
  timerCount++;
  Serial.print("timerCount= ");
  Serial.println(timerCount);

  
  /*
  digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  delay(3000);
  */

  //only send data when connected
  if(connectedWiFi){
    //Send a packet
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("RX RSSI: %u", (String)RxRssi);
    udp.printf(" - ");
    udp.printf("TX RSSI: %u", (String)TxRssi);
    udp.printf("\n");
    udp.endPacket();
  }

  //digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  //digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);                       // wait for a second
  //delay(3000);

}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connectedWiFi = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connectedWiFi = false;
          break;
    }
}
