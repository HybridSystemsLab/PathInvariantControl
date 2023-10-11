/*
 WiFiEsp example: WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.
 When a packet is received an 'ACK' packet is sent to the client on port remotePort.

 For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-client.html
*/

/***************** WIFI DEFINES ***************/
#include <WiFiEsp.h>
#include <WiFiEspUDP.h>

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(11, 4); // RX, TX
#endif

char ssid[] = "HDCL_PIC_AP2";            // your network SSID (name)
char pass[] = "";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

unsigned int localPort = 10002;  // local port to listen on

char packetBuffer[255];          // buffer to hold incoming packet
char ReplyBuffer[] = "ACK";      // a string to send back

WiFiEspUDP Udp;
/***************** END ***************/

/***************** CAR DEFINES ***************/
#include <PWMServo.h>
//define Model X pins
#define IN1 7
#define IN2 8
#define ENA 5        //  ENA pin
 
//define steer directing
 
#define FRONT 140        // adjust this value if your steer is not facing front at beginning
#define SERVO_PIN 9  //servo connect to D7

int steering_angle = FRONT; // front - angle
int velocity = 0; // 0 - 255

PWMServo head;

/***************** END ***************/

void setup() {
  /***************** WIFI SET UP ***************/
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  Serial.print("MAC ADDY: ");
  uint8_t macAddy[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(macAddy);
  for(int i = 0; i < WL_MAC_ADDR_LENGTH; i++)
  {
    Serial.print(macAddy[i], HEX);
    Serial.print(":");
  }
  Serial.println();

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
  
  Serial.print("Listening on port ");
  Serial.println(localPort);

  /***************** END WIFI SET UP ***************/

  /***************** CAR SET UP ***************/

  pinMode(ENA, OUTPUT); 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 

  head.attach(SERVO_PIN);
  head.write(steering_angle);

  /***************** END CAR SET UP ***************/
}

void loop() {

/***************** WIFI LOOP ***************/
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
//    Serial.print("Received packet of size ");
//    Serial.println(packetSize);
//    Serial.print("From ");
//    IPAddress remoteIp = Udp.remoteIP();
//    Serial.print(remoteIp);
//    Serial.print(", port ");
//    Serial.println(Udp.remotePort());

    uint8_t byte1 = Udp.read();
    steering_angle = FRONT - (byte1 - 40);
    uint8_t byte2 = Udp.read();
    velocity = byte2;
    Serial.println(byte2);
    // read the packet into packetBufffer
//    while(packetSize>0)
//    {
//      if(packetSize == 2)
//      {
//        int8_t byte1 = Udp.read();
//        Serial.print("Steering angle: ");
//        Serial.println(byte1);
//        steering_angle = FRONT - (byte1 - 40);
//      }
//
//      if(packetSize == 1)
//      {
//        uint8_t byte2 = Udp.read();
//        Serial.print("Velocity: ");
//        Serial.println(byte2);
//        velocity = byte2;
//      }
//      
//      packetSize--;
//    }
//    int len = Udp.read(packetBuffer, 255);
//    if (len > 0) {
//      packetBuffer[len] = 0;
//    }
//    Serial.println("Contents:");
//    String test1 = String(packetBuffer[0]);
//    Serial.println(test1);
//    Serial.println(packetBuffer[1]);

    // send a reply, to the IP address and port that sent us the packet we received
//    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
//    Udp.write(steering_angle);
//    Udp.endPacket();
  }
  /***************** END WIFI LOOP ***************/

  /***************** CAR LOOP ***************/
  head.write(steering_angle);
  if(velocity == 0)
  {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2,LOW);
     analogWrite(ENA,0);
  } 
  else if(velocity > 0)
  {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2,HIGH);
     analogWrite(ENA,velocity); 
  }
  else
  {
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2,LOW);
     analogWrite(ENA,velocity*-1); 
  }
  
  /***************** END CAR LOOP ***************/
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
