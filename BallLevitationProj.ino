// --- PIN DEFINITIONS (Updated to your setup) ---
const int trigPin = 9;
const int echoPin = 8;
const int fanSpeedPin = 5; // PWM pin
const int in3 = 7;         
const int in4 = 6;         

// --- PID TUNING ---
double Kp = 2.0;   
double Ki = 0.2;  
double Kd = 0.50;   

// --- ALWAYS-ON SETTINGS ---
// Adjust MIN_FAN_SPEED so the fan spins very slowly but never stops.
// Usually between 70 and 100 for most 12V fans.
const int MIN_FAN_SPEED = 20; 
const int MAX_FAN_SPEED = 255;

// --- CONTROL VARIABLES ---
double setpoint = -15.0; 
double input, output, error, lastError, integral, derivative;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(fanSpeedPin, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set direction once
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  lastTime = millis();
}

void loop() {
  // 1. MEASURE DISTANCE
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); 
  double distance = duration * 0.034 / 2;

  // SAFETY FILTER: If sensor glitces, keep the last good reading
  if (distance > 0 && distance < 100) {
    input = distance;
  }

  // 2. PID CALCULATION
  unsigned long currentTime = millis();
  double timeChange = (double)(currentTime - lastTime) / 1000.0;
  if (timeChange <= 0) timeChange = 0.01; // Prevent division by zero

  error = input - setpoint; 
  integral += error * timeChange;
  
  // Prevent Integral Windup
  integral = constrain(integral, -50, 50);

  derivative = (error - lastError) / timeChange;
  
  // Calculate raw PID output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // 3. ALWAYS-ON LOGIC
  // Instead of 0-255, we constrain it between MIN_FAN_SPEED and 255
  int fanPWM = constrain(output, MIN_FAN_SPEED, MAX_FAN_SPEED);

  analogWrite(fanSpeedPin, fanPWM);

  // 4. UPDATE FOR NEXT LOOP
  lastError = error;
  lastTime = currentTime;

  // DEBUGGING
  Serial.print("Dist: "); Serial.print(input);
  Serial.print("cm | PWM: "); Serial.println(fanPWM);
  
  delay(5); 
}