#include <SoftwareSerial.h>  // Library for serial communication
#include <DHT.h>  // Library for temperature sensor

SoftwareSerial BT_Serial(2, 3); // RX, TX

#define enA 10 // Enable1 L298 Pin enA 
#define in1 9  // Motor1 L298 Pin in1 
#define in2 8  // Motor1 L298 Pin in2 
#define in3 7  // Motor2 L298 Pin in3 
#define in4 6  // Motor2 L298 Pin in4 
#define enB 5  // Enable2 L298 Pin enB 

#define servo 12

#define R_S A1  // IR sensor Right
#define L_S A0 // IR sensor Left

#define LIR A4
#define RIR A5 

#define echo A2    // Echo pin
#define trigger A3 // Trigger pin

const int buzzerPin = 11;

#define DHTPIN 4 
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

int distance_L, distance_F = 30, distance_R;
long distance;
int set = 20;

int bt_ir_data; // variable to receive data from the serial port and IR remote
int Speed = 150;  
int mode = 0;

void setup() {
  pinMode(R_S, INPUT);  
  pinMode(L_S, INPUT);
  pinMode(echo, INPUT); // declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output  
  pinMode(buzzerPin, OUTPUT);
  pinMode(enA, OUTPUT); 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);   
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  Serial.begin(9600); // start serial communication at 9600bps
  BT_Serial.begin(9600); 
  dht.begin();
  pinMode(servo, OUTPUT);

  // Initialize servo position
  for (int angle = 70; angle <= 140; angle += 5) { servoPulse(servo, angle); }
  for (int angle = 140; angle >= 0; angle -= 5) { servoPulse(servo, angle); }
  for (int angle = 0; angle <= 70; angle += 5) { servoPulse(servo, angle); }
  delay(500);
}

void loop() {  
  // Read Bluetooth commands
  handleBluetoothCommands();

  analogWrite(enA, Speed);
  analogWrite(enB, Speed);

  // Read sensor data
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  distance_F = Ultrasonic_read();
  
  // Only send data every 2 seconds
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= 1000) {
    sendSensorData(t, h, distance_F);
    lastSendTime = millis();
  }

  // Process modes without blocking delays
  switch (mode) {
    case 0: Stop(); break; // Main Screen
    case 1: handleManualMode(); break; // Manual Control
    case 2: handleObstacleAvoidance(); break; // Obstacle Avoidance
    case 3: handleLineFollower(); break; // Line Follower
    case 4: handleVoiceControl(); break; // Voice Control
  }

  delay(10); // Add a small delay for loop stability
}

// Function to handle Bluetooth commands
void handleBluetoothCommands() {
  if (BT_Serial.available() > 0) {
    bt_ir_data = BT_Serial.read(); 
    Serial.println(bt_ir_data);     
    if (bt_ir_data > 20) { Speed = bt_ir_data; }      
    
    // Update mode based on received data
    if (bt_ir_data == 1) { mode = 1; Stop(); }
    else if (bt_ir_data == 2) { mode = 1; Stop(); }
    else if (bt_ir_data == 3) { mode = 2; Stop(); Speed = 150; }
    else if (bt_ir_data == 4) { mode = 3; Stop(); Speed = 150; }
    else if (bt_ir_data == 5) { mode = 4; Stop(); Speed = 150; }
    else if (bt_ir_data == 0) { mode = 0; Stop(); Speed = 150; }
  }
}

// Function to send sensor data
void sendSensorData(float temperature, float humidity, int distance) {
  BT_Serial.print(temperature);
  BT_Serial.print("|");
  BT_Serial.print(humidity);
  BT_Serial.print("|");
  BT_Serial.print(distance);
  BT_Serial.print("|");
  BT_Serial.print(digitalRead(LIR));
  BT_Serial.print("|");
  BT_Serial.print(digitalRead(RIR));
  BT_Serial.print("|");
}

// Function to handle manual mode
void handleManualMode() {
  switch (bt_ir_data) {
    case 6: forword(); break;      // Move forward
    case 7: turnLeft(); break;     // Turn left
    case 8: turnRight(); break;    // Turn right
    case 9: backword(); break;     // Move backward
    case 10: Stop(); break;        // Stop
    case 11: tone(buzzerPin, 1000); break; // Sound buzzer
    case 12: noTone(buzzerPin); break; // Stop buzzer
  }
}

// Function to handle obstacle avoidance mode
void handleObstacleAvoidance() {       
  distance_F = Ultrasonic_read();
  Serial.print("S=");
  Serial.println(distance_F);
  int LIRState = digitalRead(LIR);
  int RIRState = digitalRead(RIR);
  Serial.print(LIRState);
  Serial.print(RIRState);

  if (distance_F > set) {
    if (LIRState == 0 && RIRState == 0) {
      backword();
      delay(200);
      turnRight();
      delay(200);
    } else if (LIRState == 0 && RIRState == 1) {
      turnRight();
      delay(200);
    } else if (LIRState == 1 && RIRState == 0) {
      turnLeft();
      delay(200);
    }
    forword(); // Continue forward if there's no obstacle
  } else {
    Check_side();
  }
}

// Function to handle line following mode
void handleLineFollower() {
  Serial.print("Left IR: ");
  Serial.println(digitalRead(L_S));
  Serial.print("Right IR: ");
  Serial.println(digitalRead(R_S));      
  if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1)) { // Both sensors see white
    forword();
  } else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0)) { // Right sensor sees black
    turnRight();
  } else if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1)) { // Left sensor sees black
    turnLeft();
  } else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1)) { // Both sensors see black
    Stop();
  }
}

// Function to handle voice control mode
void handleVoiceControl() {
  switch (bt_ir_data) {
    case 6:{
      forword();
      if(distance_F<set){
        Stop();  
      }  
      break;
    } // Move forward
    case 7: turnLeft(); delay(400); bt_ir_data = 10; break; // Turn left and reset command
    case 8: turnRight(); delay(400); bt_ir_data = 10; break; // Turn right and reset command
    case 9: backword(); break; // Move backward and reset command
    case 10: Stop(); break; // Stop
  }
}

// Servo initialization function
void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500; // Convert angle to microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50); // Refresh cycle of servo
}

// Ultrasonic reading function
long Ultrasonic_read() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  distance = pulseIn(echo, HIGH);
  return distance / 29 / 2;
}

// Comparing distances function
void compareDistance() {
  if (distance_L > distance_R) {
    turnLeft();
    delay(350);
  } else if (distance_R > distance_L) {
    turnRight();
    delay(350);
  } else {
    backword();
    delay(300);
    turnRight();
    delay(600);
  }
}

// Checking sides function
void Check_side() {
  Stop();
  delay(100);

  for (int angle = 70; angle <= 140; angle += 5) { servoPulse(servo, angle); }
  delay(300);
  distance_L = Ultrasonic_read();
  delay(100);

  for (int angle = 140; angle >= 0; angle -= 5) { servoPulse(servo, angle); }
  delay(500);
  distance_R = Ultrasonic_read();
  delay(100);

  for (int angle = 0; angle <= 70; angle += 5) { servoPulse(servo, angle); }
  delay(300);

  compareDistance();
}




//====================================================================================
//                                   Movements
//====================================================================================

void forword(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backword(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Stop(){ 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); 
  digitalWrite(in4, LOW);
}
