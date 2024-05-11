//Torque 
volatile int currentSlit = 1; // Variable to store the current slit
volatile int limitSlit = 1;   // Variable to store the limit slit
volatile bool motorRunning = false; // Variable to track motor state




// USG 
const int trigPin = 46;
const int echoPin = 48;
unsigned long previousMillis = 0;
const long interval = 1000; // Interval in milliseconds

long duration;
int distance;



#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// Verified Pins Start 
#define ONE_WIRE_BUS 47
const int SENSOR_PIN = 47; // Arduino pin connected to DS18B20 sensor's DQ pin




float tempCelsius; // temperature in Celsius


#define TRIGGER_PIN 46
#define ECHO_PIN 48
const int buzzer = 7; //buzzer to arduino pin 7
// Verified Pins End 


#define IR_SENSOR_PIN 9
#define IR_SENSOR_2_PIN 10
#define IR_SENSOR_3_PIN 11
#define VIBRATION_SENSOR_PIN 49
#define LM393_1_PIN 3
#define LM393_2_PIN 2
#define LM393_3_PIN 1
#define HALL_EFFECT_SENSOR_PIN 7


float ds18b20Temp = 25;
float mlx90614Temp = 0;
int ultrasonicDistance = 0;
int irValue = 0;
int irValue2 = 0;
int irValue3 = 0;
int vibrationValue = 0;
int lm393_1Value = 0;
int lm393_2Value = 0;
int lm393_3Value = 0;
int hallEffectValue = 0;


// handDetect function 
unsigned long motorCStopTime = 0;
unsigned long irSensor3InactiveTime = 0;




// Sensor Instances
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Timer Variables
int scanCount = 0;
unsigned long startTime = 0;




//  MOTOR SECTION 

// Define pins for the first L298N motor controller
const int motorA1 = 25;
const int motorA2 = 27;
const int motorB1 = 29;
const int motorB2 = 31;
const int enableA = 23; // PWM pin for Motor A speed control
const int enableB = 33; // PWM pin for Motor B speed control

// Define pins for the second L298N motor controller
const int motorC1 = 37;
const int motorC2 = 39;
const int motorD1 = 41;
const int motorD2 = 43;
const int enableC = 35; // PWM pin for Motor C speed control
const int enableD = 45; // PWM pin for Motor D speed control

const unsigned long motorDuration = 5000; // Duration for each motor to run in milliseconds
const unsigned long delayBetweenMotors = 1000; // Delay between switching motors

//  MOTOR SECTION 


// LED SECTION 

int redPin1 = 22;
int greenPin1 = 24;
int bluePin1 = 26;

int redPin2 = 28;
int greenPin2 = 30;
int bluePin2 = 32;

int redPin3 = 34;
int greenPin3 = 36;
int bluePin3 = 38;

int redPin4 = 40;
int greenPin4 = 42;
int bluePin4 = 44;

const int blinkDuration = 100;
const int pauseDuration = 2000;
unsigned long previousMillisBlink = 0;
boolean ledsOn = false;



// LED SECTION 



void turnOffAllMotors() {
  // Turn off all motors by setting the pins to LOW and PWM to 0
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(enableA, 0);
  analogWrite(enableB, 0);

  digitalWrite(motorC1, LOW);
  digitalWrite(motorC2, LOW);
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, LOW);
  analogWrite(enableC, 0);
  analogWrite(enableD, 0);
}







void setup() {


// LED SECTION 
  pinMode(greenPin1, OUTPUT);
  pinMode(bluePin1, OUTPUT);
  pinMode(redPin1, OUTPUT);
  pinMode(greenPin2, OUTPUT);
  pinMode(bluePin2, OUTPUT);
  pinMode(redPin2, OUTPUT);
  pinMode(greenPin3, OUTPUT);
  pinMode(bluePin3, OUTPUT);
  pinMode(redPin3, OUTPUT);
  pinMode(greenPin4, OUTPUT);
  pinMode(bluePin4, OUTPUT);
  pinMode(redPin4, OUTPUT);
//LED SECTION




//  MOTOR SECTION 
// Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(motorC1, OUTPUT);
  pinMode(motorC2, OUTPUT);
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(enableC, OUTPUT);
  pinMode(enableD, OUTPUT);

// Turn off all motors
  turnOffAllMotors(); // Turn off all motors during setup
//  MOTOR SECTION 


  // Setting up the buzzer pin
  pinMode(buzzer, OUTPUT);

  // Turning off the buzzer initially
  digitalWrite(buzzer, LOW);


 // Initialize sensor pins as inputs
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_2_PIN, INPUT);
  pinMode(IR_SENSOR_3_PIN, INPUT);
  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  pinMode(LM393_1_PIN, INPUT);
  pinMode(LM393_2_PIN, INPUT);
  pinMode(LM393_3_PIN, INPUT);
  pinMode(HALL_EFFECT_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LM393_1_PIN), countSlits, RISING); // Attaching interrupt to the pin

  Serial.begin(9600);
  sensors.begin();
  mlx.begin();


  
}

// Torque Functions 
void moveToSlit(int x) {
  limitSlit = x; // Set the limit slit

  // Determine the direction of movement for the motor
  if (limitSlit > currentSlit) {
    motorRunning = true;
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    analogWrite(enableB, 200); // Set the motor speed (adjust as needed)
  } else if (limitSlit < currentSlit) {
    motorRunning = true;
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    analogWrite(enableB, 200); // Set the motor speed (adjust as needed)
  } else {
    motorRunning = false;
    analogWrite(enableB, 0); // Disable motor if already at the limit
  }
}

void countSlits() {
  if (motorRunning) {
    if (currentSlit < limitSlit) {
      currentSlit++;
    } else if (currentSlit > limitSlit) {
      currentSlit--;
    }

    if (currentSlit == limitSlit) {
      motorRunning = false;
      analogWrite(enableB, 0); // Disable motor when limit is reached
      Serial.println("Limit Reached");
    }

    Serial.print("Slit Count: ");
    Serial.println(currentSlit); // Output the slit count to the serial monitor
  }
}

















// USG FUNCTIONS 
void triggerUltrasonicSensor() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
}

void readUltrasonicSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    triggerUltrasonicSensor();
    unsigned long startMillis = millis();
    while (digitalRead(ECHO_PIN) == LOW) {
      if (millis() - startMillis > 1000) // Timeout if no response within 1 second
        break;
    }
    startMillis = millis();
    while (digitalRead(ECHO_PIN) == HIGH) {
      if (millis() - startMillis > 1000) // Timeout if no response within 1 second
        break;
    }
    unsigned long duration = millis() - startMillis;
    int distance = duration * 0.034 / 2; // Calculate distance in centimeters
    Serial.print("Ultrasonic Distance: ");
    Serial.println(distance);
  }
}














// MOTOR SECTION -FUNCTIONS 
void turnOffMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(enableA, 0);
  analogWrite(enableB, 0);

  digitalWrite(motorC1, LOW);
  digitalWrite(motorC2, LOW);
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, LOW);
  analogWrite(enableC, 0);
  analogWrite(enableD, 0);


}

// Function to rotate Motor A in the clockwise direction
void rotateMotorA_Clockwise() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(enableA, 200); // Set PWM for motor A speed control
}

// Function to rotate Motor A in the anticlockwise direction
void rotateMotorA_AntiClockwise() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(enableA, 200); // Set PWM for motor A speed control
}

// Function to apply brake to Motor A
void applyBrakeMotorA() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  analogWrite(enableA, 0); // Stop motor A by setting PWM to 0
}

// Function to stop Motor A
void stopMotorA() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  analogWrite(enableA, 0); // Stop motor A by setting PWM to 0
}




// Function to rotate Motor B in the clockwise direction
void rotateMotorB_Clockwise() {
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(enableB, 200); // Set PWM for motor B speed control
}

// Function to rotate Motor B in the anticlockwise direction
void rotateMotorB_AntiClockwise() {
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(enableB, 200); // Set PWM for motor B speed control
}

// Function to apply brake to Motor B
void applyBrakeMotorB() {
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
  analogWrite(enableB, 0); // Stop motor B by setting PWM to 0
}

// Function to stop Motor B
void stopMotorB() {
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(enableB, 0); // Stop motor B by setting PWM to 0
}




// Function to rotate Motor C in the clockwise direction
void rotateMotorC_Clockwise() {
  digitalWrite(motorC1, HIGH);
  digitalWrite(motorC2, LOW);
  analogWrite(enableC, 200); // Set PWM for motor C speed control
}

// Function to rotate Motor C in the anticlockwise direction
void rotateMotorC_AntiClockwise() {
  digitalWrite(motorC1, LOW);
  digitalWrite(motorC2, HIGH);
  analogWrite(enableC, 200); // Set PWM for motor C speed control
}

// Function to apply brake to Motor C
void applyBrakeMotorC() {
  digitalWrite(motorC1, HIGH);
  digitalWrite(motorC2, HIGH);
  analogWrite(enableC, 0); // Stop motor C by setting PWM to 0
}

// Function to stop Motor C
void stopMotorC() {
  digitalWrite(motorC1, LOW);
  digitalWrite(motorC2, LOW);
  analogWrite(enableC, 0); // Stop motor C by setting PWM to 0
}

// Function to rotate Motor D in the clockwise direction
void rotateMotorD_Clockwise() {
  digitalWrite(motorD1, HIGH);
  digitalWrite(motorD2, LOW);
  analogWrite(enableD, 200); // Set PWM for motor D speed control
}

// Function to rotate Motor D in the anticlockwise direction
void rotateMotorD_AntiClockwise() {
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, HIGH);
  analogWrite(enableD, 200); // Set PWM for motor D speed control
}

// Function to apply brake to Motor D
void applyBrakeMotorD() {
  digitalWrite(motorD1, HIGH);
  digitalWrite(motorD2, HIGH);
  analogWrite(enableD, 0); // Stop motor D by setting PWM to 0
}

// Function to stop Motor D
void stopMotorD() {
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, LOW);
  analogWrite(enableD, 0); // Stop motor D by setting PWM to 0
}


void runMotorForDuration(int pin1, int pin2, int en) {
  // Run motor for the specified duration
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  analogWrite(en, 200); // Set PWM for motor speed control
  
  delay(motorDuration);
  
  // Stop the motor after the duration
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  analogWrite(en, 0); // Stop motor by setting PWM to 0
  
  // Delay between switching motors
  delay(delayBetweenMotors);
}

// MOTOR SECTION 






//LED SECTION- FUNCTIONS 
void LED1White() {
  analogWrite(redPin1, 255);
  analogWrite(greenPin1, 255);
  analogWrite(bluePin1, 255);
}

void LED1Red() {
  analogWrite(redPin1, 255);
  analogWrite(greenPin1, 0);
  analogWrite(bluePin1, 0);
}

void LED1Green() {
  analogWrite(redPin1, 0);
  analogWrite(greenPin1, 255);
  analogWrite(bluePin1, 0);
}

void LED1Blue() {
  analogWrite(redPin1, 0);
  analogWrite(greenPin1, 0);
  analogWrite(bluePin1, 255);
}

void LED1Off() {
  analogWrite(redPin1, 0);
  analogWrite(greenPin1, 0);
  analogWrite(bluePin1, 0);
}


void LED2White() {
  analogWrite(redPin2, 255);
  analogWrite(greenPin2, 255);
  analogWrite(bluePin2, 255);
}

void LED2Red() {
  analogWrite(redPin2, 255);
  analogWrite(greenPin2, 0);
  analogWrite(bluePin2, 0);
}

void LED2Green() {
  analogWrite(redPin2, 0);
  analogWrite(greenPin2, 255);
  analogWrite(bluePin2, 0);
}

void LED2Blue() {
  analogWrite(redPin2, 0);
  analogWrite(greenPin2, 0);
  analogWrite(bluePin2, 255);
}

void LED2Off() {
  analogWrite(redPin2, 0);
  analogWrite(greenPin2, 0);
  analogWrite(bluePin2, 0);
}



void LED3White() {
  analogWrite(redPin3, 255);
  analogWrite(greenPin3, 255);
  analogWrite(bluePin3, 255);
}

void LED3Red() {
  analogWrite(redPin3, 255);
  analogWrite(greenPin3, 0);
  analogWrite(bluePin3, 0);
}

void LED3Green() {
  analogWrite(redPin3, 0);
  analogWrite(greenPin3, 255);
  analogWrite(bluePin3, 0);
}

void LED3Blue() {
  analogWrite(redPin3, 0);
  analogWrite(greenPin3, 0);
  analogWrite(bluePin3, 255);
}

void LED3Off() {
  analogWrite(redPin3, 0);
  analogWrite(greenPin3, 0);
  analogWrite(bluePin3, 0);
}

void LED4White() {
  analogWrite(redPin4, 255);
  analogWrite(greenPin4, 255);
  analogWrite(bluePin4, 255);
}

void LED4Red() {
  analogWrite(redPin4, 255);
  analogWrite(greenPin4, 0);
  analogWrite(bluePin4, 0);
}

void LED4Green() {
  analogWrite(redPin4, 0);
  analogWrite(greenPin4, 255);
  analogWrite(bluePin4, 0);
}

void LED4Blue() {
  analogWrite(redPin4, 0);
  analogWrite(greenPin4, 0);
  analogWrite(bluePin4, 255);
}

void LED4Off() {
  analogWrite(redPin4, 0);
  analogWrite(greenPin4, 0);
  analogWrite(bluePin4, 0);
}




void LEDOn() {
  analogWrite(redPin1, 255);
  analogWrite(greenPin1, 255);
  analogWrite(bluePin1, 255);

  analogWrite(redPin2, 255);
  analogWrite(greenPin2, 255);
  analogWrite(bluePin2, 255);

  analogWrite(redPin3, 255);
  analogWrite(greenPin3, 255);
  analogWrite(bluePin3, 255);

  analogWrite(redPin4, 255);
  analogWrite(greenPin4, 255);
  analogWrite(bluePin4, 255);

  ledsOn = true;
}

void LEDOff() {
  analogWrite(redPin1, 0);
  analogWrite(greenPin1, 0);
  analogWrite(bluePin1, 0);

  analogWrite(redPin2, 0);
  analogWrite(greenPin2, 0);
  analogWrite(bluePin2, 0);

  analogWrite(redPin3, 0);
  analogWrite(greenPin3, 0);
  analogWrite(bluePin3, 0);

  analogWrite(redPin4, 0);
  analogWrite(greenPin4, 0);
  analogWrite(bluePin4, 0);

  ledsOn = false;
}

void runBlink() {
  unsigned long currentMillisBlink = millis();

  if (!ledsOn) {
    if (currentMillisBlink - previousMillisBlink >= pauseDuration) {
      previousMillisBlink = currentMillisBlink;
      LEDOn();
    }
  } else {
    if (currentMillisBlink - previousMillisBlink >= blinkDuration) {
      previousMillisBlink = currentMillisBlink;
      LEDOff();
    }
  }
}
// LED SECTION- FUNCTIONS 


void triggerUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void readDistance() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    triggerUltrasonic();
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    //Serial.print("Distance: ");
    //Serial.println(distance);
  }
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;


readDistance();


if (currentTime - previousMillis >= interval) {
    previousMillis = currentTime;

    
    
  sensors.requestTemperatures();            // send the command to get temperatures
  tempCelsius = sensors.getTempCByIndex(0); // read temperature in Celsius 
    
    
    float ds18b20Temp = sensors.getTempCByIndex(0);
    float mlx90614Temp = mlx.readObjectTempC();
    int ultrasonicDistance = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
// Read the echo signal






    // Read IR sensor values for motion detection
    int irValue = analogRead(IR_SENSOR_PIN);
    int irValue2 = analogRead(IR_SENSOR_2_PIN);
    int irValue3 = analogRead(IR_SENSOR_3_PIN);
    int vibrationValue = digitalRead(VIBRATION_SENSOR_PIN);
    int lm393_1Value = digitalRead(LM393_1_PIN);
    int lm393_2Value = digitalRead(LM393_2_PIN);
    int lm393_3Value = digitalRead(LM393_3_PIN);
    int hallEffectValue = digitalRead(HALL_EFFECT_SENSOR_PIN);

    Serial.print("#- ");
    Serial.print(scanCount);


        Serial.print(" DT- ");
    Serial.print(ds18b20Temp);
    Serial.print(" MT- ");
    Serial.print(mlx90614Temp);
    Serial.print(" USG- ");
    Serial.print(distance);
    // Print sensor readings to Serial Monitor
    Serial.print(" 1V:= ");
    Serial.println(irValue);

    Serial.print(" 2V:= ");
    Serial.println(irValue2);

    Serial.print(" 3V:= ");
    Serial.println(irValue3);
    Serial.print(" Vbn =");
    Serial.print(vibrationValue);
    Serial.print(" . ");
    Serial.print(lm393_1Value);
    Serial.print(" . ");
    Serial.print(lm393_2Value);
    Serial.print(" . ");
    Serial.print(lm393_3Value);
    Serial.print(" . ");
    Serial.println(hallEffectValue);

    scanCount++;

    if (scanCount >= 20) {
      Serial.println("Scanning completed for 20 minutes.");
      while (true) {
        // Loop indefinitely after 20 scans
      }
    }
  }














  if (elapsedTime < 1200000) { // Run for 20 minutes (1200000 milliseconds)
    // Calculate which minute it is



    int currentMinute = elapsedTime / 60000; // Each minute is 60000 milliseconds

    // Motor control logic based on the current minute
    if (currentMinute == 0) {
      // Code for the first minute
      

    runBlink();
    moveToSlit(4); // Change this value to set the desired limit slit

    

  
  

   


    } else if (currentMinute == 1) {
      // Code for the second minute
      
    runBlink();
    moveToSlit(2); // Change this value to set the desired limit slit
    




    } else if (currentMinute == 2) {
      // Code for the third minute

    runBlink();
    
      
        




    } else if (currentMinute == 3) {
      // Code for the fourth minute

runBlink();


    } else if (currentMinute == 4) {
      // Code for the fifth minute





    } else if (currentMinute == 5) {
      // Code for the sixth minute




    } else if (currentMinute == 6) {
      // Code for the seventh minute





    } else if (currentMinute == 7) {
      // Code for the eighth minute





    } else if (currentMinute == 8) {
      // Code for the ninth minute






    } else if (currentMinute == 9) {
      // Code for the tenth minute





    } else if (currentMinute == 10) {
      // Code for the eleventh minute






    } else if (currentMinute == 11) {
      // Code for the twelfth minute






    } else if (currentMinute == 12) {
      // Code for the thirteenth minute







    } else if (currentMinute == 13) {
      // Code for the fourteenth minute







    } else if (currentMinute == 14) {
      // Code for the fifteenth minute






    } else if (currentMinute == 15) {
      // Code for the sixteenth minute






    } else if (currentMinute == 16) {
      // Code for the seventeenth minute





    } else if (currentMinute == 17) {
      // Code for the eighteenth minute






    } else if (currentMinute == 18) {
      // Code for the nineteenth minute






    } else if (currentMinute == 19) {
      // Code for the twentieth minute





    }
  } else {
    // After 20 minutes, restart the sequence
    startTime = millis(); // Reset the start time
  }
}




