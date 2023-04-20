#include <WiFi.h>
#include <WiFiUdp.h>

// Replace with your network credentials
const char* ssid = "memristor";
const char* password = "ftnmemristor";

// UDP server parameters
WiFiUDP udp;
unsigned int localUdpPort = 8888;

// Counter parameters
unsigned long counter = 0;
unsigned long lastTime = 0;
unsigned long interval = 1000; // milliseconds
unsigned long UDP_TX_PACKET_MAX_SIZE = 32;

void setup() {
  Serial.begin(115200, SERIAL_8N1);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Start UDP server
  if (!udp.begin(localUdpPort)) {
    Serial.println("Failed to start UDP server");
    while (1) {}
  }
  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("UDP server started on port ");
  Serial.println(localUdpPort);
}

void loop() {
  // Send counter value every "interval" milliseconds
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    // Convert counter value to string and send via UDP
    char message[32];
    sprintf(message, "%lu", counter);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(message);
    udp.endPacket();
    Serial.print("Sent message: ");
    Serial.println(message);

    // Increment counter and update lastTime
    counter++;
    lastTime = currentTime;
  }

  // Check for incoming UDP messages
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
    udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.print("Received message: ");
    Serial.println(packetBuffer);
  }
}
