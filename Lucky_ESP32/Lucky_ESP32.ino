#include <WiFi.h>
#include <WiFiUdp.h>

// --- SETTINGS ---
const char* ssid = "Verizon_L967JT";
const char* password = "poet-ohm7-align";
unsigned int localPort = 8888;

WiFiUDP udp;
IPAddress laptopIP;
unsigned int laptopPort;
bool hasLaptopIP = false; // Flag to know when we can start sending data back
const int BLUE_LED = 2; 

void setup() {
  pinMode(BLUE_LED, OUTPUT);
  
  Serial.begin(115200); // Debug to Mac via USB

  // Pins 16 (RX) and 17 (TX) for Teensy 4.1
  Serial2.begin(921600, SERIAL_8N1, 16, 17);
  Serial2.setTimeout(10); // Makes Serial communication much snappier

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
    delay(200);
    Serial.print(".");
  }

  digitalWrite(BLUE_LED, HIGH); 
  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  udp.begin(localPort);
}

void loop() {
  // A. FROM MAC (UDP) -> TO TEENSY (Serial2)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    laptopIP = udp.remoteIP();
    laptopPort = udp.remotePort();
    hasLaptopIP = true; // We now know where to send feedback!
    
    char buf[255];
    int len = udp.read(buf, 255);
    buf[len] = 0;
    
    // Clean up the string and send to Teensy
    String cmd = String(buf);
    cmd.trim(); 
    Serial2.println(cmd); 
    
    Serial.print("UDP -> Teensy: "); Serial.println(cmd);
  }

  // B. FROM TEENSY (Serial2) -> TO MAC (UDP)
  if (Serial2.available()) {
    String feedback = Serial2.readStringUntil('\n');
    feedback.trim();
    
    Serial.print("Teensy -> UDP: "); Serial.println(feedback);
    
    if (hasLaptopIP) {
      udp.beginPacket(laptopIP, laptopPort);
      udp.print(feedback);
      udp.endPacket();
    }
  }
}