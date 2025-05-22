#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Ultrasonic Sensor Pins
const int trigPin1 = 2;
const int echoPin1 = 3;
const int trigPin2 = 4;
const int echoPin2 = 5;
const int trigPin3 = 6;
const int echoPin3 = 7;

// Buzzer and Button Pins
const int buzzerPin = 8;
const int buttonPin = 9;

// Distance threshold in cm
const int threshold = 20;

// GPS and GSM Software Serial
SoftwareSerial gpsSerial(10, 11); // RX, TX for NEO-6M
SoftwareSerial gsmSerial(12, 13); // RX, TX for A767X

TinyGPSPlus gps;

String phoneNumber = ""; // Replace with your phone number

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("Initializing System...");

  // Check GSM connection
  gsmSerial.println("AT");
  delay(1000);
  if (gsmSerial.available()) {
    String response = gsmSerial.readString();
    if (response.indexOf("OK") != -1) {
      Serial.println("GSM Module Connected!");
    } else {
      Serial.println("GSM Module not responding!");
    }
  } else {
    Serial.println("GSM Serial not available!");
  }

  Serial.println("Waiting for GPS fix...");
}

void loop() {
  // Check distances and buzz accordingly
  float distance1 = getDistance(trigPin1, echoPin1);
  float distance2 = getDistance(trigPin2, echoPin2);
  float distance3 = getDistance(trigPin3, echoPin3);

  if (distance1 < threshold) {
    tone(buzzerPin, 500);
    delay(200);
    noTone(buzzerPin);
  }
  if (distance2 < threshold) {
    tone(buzzerPin, 1000);
    delay(200);
    noTone(buzzerPin);
  }
  if (distance3 < threshold) {
    tone(buzzerPin, 1500);
    delay(200);
    noTone(buzzerPin);
  }

  // Read and display GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Waiting for GPS Fix...");
  }

  // Check button press
  if (digitalRead(buttonPin) == LOW) {
    sendLocation();
    delay(3000);  // Debounce and avoid multiple messages
  }

  delay(500);
}

// Function to measure distance from ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  
  return distance;
}

// Function to send GPS location via SMS
void sendLocation() {
  if (gps.location.isValid()) {
    String message = "Help Needed! Location: ";
    message += "Lat: " + String(gps.location.lat(), 6);
    message += ", Lon: " + String(gps.location.lng(), 6);

    Serial.println("Sending SMS...");
    Serial.println("Message: " + message);

    // Send AT Commands to GSM
    gsmSerial.println("AT");
    delay(1000);
    gsmSerial.println("AT+CMGF=1");  // Set SMS text mode
    delay(1000);
    gsmSerial.print("AT+CMGS=\"");
    gsmSerial.print(phoneNumber);
    gsmSerial.println("\"");
    delay(1000);
    gsmSerial.println(message);
    delay(500);
    gsmSerial.write(26);  // Ctrl+Z to send
    delay(3000);

    Serial.println("SMS Sent!");
  } else {
    Serial.println("Cannot send SMS: GPS fix not available.");
  }
}