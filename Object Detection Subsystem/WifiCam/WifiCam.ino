#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <ESP32Servo.h>

const char* WIFI_SSID = "ESP32-CAM";
const char* WIFI_PASS = "password123";

// Output pins
const int solenoidPin = 12;
const int ledPin = 13;
const int panPin = 14;
const int tiltPin1 = 15;
//const int tiltPin2 = 2;  // Additional inverted tilt servo

// Servo objects
Servo panServo;
Servo tiltServo1;
//Servo tiltServo2;

// Current positions
int currentPan = 90;
int currentTilt = 90;

// Alert system
unsigned long alertStartTime = 0;
bool alertActive = false;
bool ledOn = false;
const unsigned long ALERT_DURATION = 5000;
const unsigned long SOLENOID_DURATION = 2000;

WebServer server(80);
static auto hiRes = esp32cam::Resolution::find(800, 600);

void serveJpg() {
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    server.send(503, "", "");
    return;
  }
  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

void handleControl() {
  // Handle servo commands
  if (server.hasArg("pan") && server.hasArg("tilt")) {
    currentPan = constrain(server.arg("pan").toInt(), 0, 180);
    currentTilt = constrain(server.arg("tilt").toInt(), 0, 180);
    
    panServo.write(currentPan);
    tiltServo1.write(currentTilt);
    //tiltServo2.write(180 - currentTilt);  // Inverted tilt servo
  }
  
  // Handle LED command
  if (server.hasArg("led")) {
    ledOn = server.arg("led") == "1";
    digitalWrite(ledPin, ledOn ? HIGH : LOW);
  }
  
  // Handle alert command
  if (server.hasArg("alert") && !alertActive) {
    alertActive = true;
    alertStartTime = millis();
    digitalWrite(solenoidPin, HIGH);
    Serial.println("Alert activated");
  }
  
  // Handle alert cancellation
  if (server.hasArg("cancel_alert") && alertActive) {
    alertActive = false;
    digitalWrite(solenoidPin, LOW);
    Serial.println("Alert cancelled");
  }
  
  server.send(200, "text/plain", "OK");
}

void updateAlertSystem() {
  if (alertActive) {
    unsigned long currentTime = millis();
    
    // Turn off solenoid after duration
    if ((currentTime - alertStartTime) > SOLENOID_DURATION && 
        digitalRead(solenoidPin)) {
      digitalWrite(solenoidPin, LOW);
      Serial.println("Solenoid deactivated");
    }
    
    // Turn off alert after duration
    if ((currentTime - alertStartTime) > ALERT_DURATION) {
      alertActive = false;
      Serial.println("Alert system reset");
    }
  }
  
  // Keep LED state as set by commands
  digitalWrite(ledPin, ledOn ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize outputs
  pinMode(solenoidPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);
  digitalWrite(ledPin, LOW);
  
  // Initialize servos
  panServo.attach(panPin);
  tiltServo1.attach(tiltPin1);
  tiltServo2.attach(tiltPin2);
  panServo.write(currentPan);
  tiltServo1.write(currentTilt);
  //tiltServo2.write(180 - currentTilt);  // Initialize inverted
  
  // Camera setup
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(hiRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);
    Camera.begin(cfg);
  }
  
  // WiFi Access Point setup
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Server endpoints
  server.on("/cam-hi.jpg", serveJpg);
  server.on("/control", handleControl);
  server.begin();
}

void loop() {
  server.handleClient();
  updateAlertSystem();
  
  // Small delay to prevent watchdog triggers
  delay(10);
}
