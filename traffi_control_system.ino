#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize the LCD with I2C address 0x27 and 16x2 size
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pins for the components
const int trigPin = 3;         // Ultrasonic sensor trig pin
const int echoPin = 4;         // Ultrasonic sensor echo pin
const int ldrPin = A0;     
const int redLedPin = 6;       
const int yellowLedPin = 11;     
const int greenLedPin = 5;     
const int buzzer=2; 

// Variables for distance and button state
long duration;
int distance;

void setup() {
  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(buttonPin, INPUT);  // Use internal pull-up resistor
  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buzzer,OUTPUT);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {

  // Measure distance using the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;  // Calculate distance in cm
  Serial.print("Distance : ");
  Serial.println(distance);
  //digitalWrite(redLedPin,HIGH);
  // Display distance on the LCD if the button is pressed
    lcd.setCursor(0, 0);
    lcd.print(" Distance: ");
    lcd.print(distance);
    lcd.print(" cm");

  // Read LDR value to adjust LED brightness
  int ldrValue = analogRead(ldrPin);
  int brightness = map(ldrValue, 0, 1023, 0, 255);  // Map LDR value to brightness range
  Serial.print("brightness : ");
  Serial.println(brightness);
  // Check distance and control LEDs
  if (distance > 10) {  // If obstacle is detected within a threshold
    analogWrite(redLedPin, brightness);
    delay(3000);
    analogWrite(redLedPin, 0);
    analogWrite(yellowLedPin, brightness);
    delay(3000);
    analogWrite(yellowLedPin, 0);
    analogWrite(greenLedPin, brightness);
    delay(3000);
    analogWrite(greenLedPin, 0);
  } else {  // Traffic system execution
    digitalWrite(buzzer,HIGH);
    analogWrite(redLedPin, brightness);  // Blink red LED based on brightness
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    delay(2000);
    digitalWrite(buzzer,LOW);
  }
}
