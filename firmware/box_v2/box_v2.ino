#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h>


// Define the pins for the motors and drivers
const int stepPin1 = 19;  
const int dirPin1 = 15;  
const int enablePin = 16;  
const int stepPin2 = 12;   //D6 
const int dirPin2 = 14;   //D5

// Define the maximum speed and acceleration of the motors
const float maxSpeed = 200; // in steps per second
const float acceleration = 200; // in steps per second per second

// Define the I2C address and dimensions of the OLED display
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 32

// Initialize the stepper motors with the pin connections
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

// Initialize the OLED display object
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Initialize the sensor pin
const int sensorPin = 36;
int motor_enable = 1;

// Initialize the object counter
int objectCount = 0;
int sensorData = 0;

unsigned long previousSensorTime = 0;
unsigned int object_measured = 0;
unsigned long previousWiFiTime = 0;
unsigned long previousDisplayTime = 0;

const unsigned long sensorInterval = 20; // interval in milliseconds
const unsigned long WiFiInterval = 1500;
const unsigned long WiFiInterval2 = 2000;
const unsigned long displayInterval = 1000;

//SERVER SETUP
// Replace with your network credentials
const char* ssid     = "memristor";
const char* password = "ftnmemristor";

// UDP server parameters
WiFiUDP udp;
unsigned int localUdpPort = 8888;
unsigned long UDP_TX_PACKET_MAX_SIZE = 32;


void setup() {
  // Set up the motor driver pins
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  Serial.begin(115200);

  // Wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  // Set up the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);        
  display.print("Memristor Robotics");
  display.display();
  delay(2000);
  display.clearDisplay();

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

  Serial.println("Waiting for 1 to start loop.");

  // Wait for "1" to be received
  bool received = false;
  while (!received) {
    if (udp.parsePacket()) {
      char buffer[128];
      Serial.print("Received flag: ");
      Serial.println(buffer);
      udp.read(buffer, 128);
      buffer[1] = 0;
      if (strcmp(buffer, "1") == 0) {
        received = true;
      }
    }
  }

  // Set up the motor properties
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(acceleration);
  stepper2.setMaxSpeed(1500);
  stepper2.setAcceleration(1500);
  
  Serial.println("Starting loop.");
}


void loop() {
  unsigned long currentTime = millis();
  sensorData = analogRead(sensorPin);
  if(sensorData > 1500 && sensorData < 3500)
    {
      objectCount++;
      delay(300);
    }
  if (currentTime - previousDisplayTime >= displayInterval) 
    {
      Serial.print("Sensor Data: ");
      Serial.println(sensorData);

      display.clearDisplay();     
      display.setCursor(0, 10);        
      display.print("Object count: ");
      display.setCursor(80, 10);
      display.print(objectCount);
      display.display();

      // send points via udp
      char message[32];
      sprintf(message, "%lu", objectCount);
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print(message);
      udp.endPacket();
      Serial.print("Sent message: ");
      Serial.println(message);

      previousDisplayTime = currentTime;
    }

  // Check for incoming UDP messages
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
    udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    motor_enable = packetBuffer[0] - '0';
    Serial.print("Received message: ");
    Serial.println(packetBuffer);
  }

  // check if motors should be turned off
  if (motor_enable)
  {
    // Move the first motor continuously
    if (!stepper1.distanceToGo()) 
      {
        stepper1.moveTo(-100000);
      }
    stepper1.run();
  
    // Move the second motor back and forth by 20 degrees
    static bool direction = true;
    static int currentPosition = 0;
    const int targetPosition = direction ? currentPosition + 20 : currentPosition - 20;
    stepper2.moveTo(targetPosition);
  
    if (abs(stepper2.distanceToGo()) < 10) 
      {
        currentPosition = targetPosition;
        direction = !direction;
      }
    stepper2.run();    
  }
  else 
  {
    stepper1.enableOutputs();
    stepper2.enableOutputs();
  }
}
