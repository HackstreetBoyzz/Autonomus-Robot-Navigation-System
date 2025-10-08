#include <ArduinoBLE.h>
#include <Servo.h>

// enums for robot modes and directions
enum RobotMode {
  MANUAL = 0,
  OBSTACLE_AVOIDANCE = 1,
  LINE_TRACKING = 2
};

enum Direction {
  FORWARD = 0x01,   // 0b00000001
  BACKWARD = 0x02,  // 0b00000010
  LEFT = 0x04,      // 0b00000100
  RIGHT = 0x08,     // 0b00001000
  STOP = 0x10       // 0b00010000
};

// structs to keep motor and sensor data
struct MotorPins {
  uint8_t ena;
  uint8_t in1;
  uint8_t in2;
  uint8_t enb;
  uint8_t in3;
  uint8_t in4;
  uint8_t speed;
};

struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
  long lastDistance;
};

struct LineTrackingSensors {
  uint8_t leftPin;
  uint8_t middlePin;
  uint8_t rightPin;
  uint8_t sensorState;
  uint8_t lastState;      // NEW: Track previous state
  uint8_t lineSpeed;      // NEW: Dedicated line tracking speed
};

struct RobotConfig {
  uint16_t obstacleDistance;
  uint16_t scanDelay;
  uint16_t turnDelay;
  uint8_t servoPin;
};

struct RobotState {
  RobotMode currentMode;
  Direction currentDirection;
  bool isConnected;
  unsigned long lastModeUpdate;
  unsigned long lastAdvertiseTime;
  unsigned long lastHeartbeat;
};

// making objects for everything so we can use easily for easy readability 
MotorPins motors = {9, 8, 7, 10, 6, 5, 180};
UltrasonicSensor ultrasonic = {A1, A0, 0};
LineTrackingSensors lineTracking = {4, 3, 2, 0, 0xFF, 100};
RobotConfig config = {70, 300, 500, 11};
RobotState robotState = {MANUAL, STOP, false, 0, 0, 0};

Servo myservo;

// BLE stuff for bluetooth connection
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEStringCharacteristic rxCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 20);
BLEStringCharacteristic txCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLERead | BLENotify, 50);

// declaring functions before using them
void initializeMotorPins(MotorPins *motor);
void initializeSensorPins(UltrasonicSensor *us, LineTrackingSensors *lt);
void initializeBLE();
void handleBTCommand(char command);
void setMotorDirection(MotorPins *motor, Direction dir);
void stopMotors(MotorPins *motor);
long getDistance(UltrasonicSensor *sensor);
void runObstacleAvoidance(MotorPins *motor, UltrasonicSensor *sensor, RobotConfig *cfg);
void runLineTracking(MotorPins *motor, LineTrackingSensors *lt);
uint8_t readLineTrackingSensors(LineTrackingSensors *lt);
bool safeBLEWrite(const char* message);

//  BITWISE HELPER MACROS 
#define SET_BIT(byte, bit)    ((byte) |= (1 << (bit)))
#define CLEAR_BIT(byte, bit)  ((byte) &= ~(1 << (bit)))
#define CHECK_BIT(byte, bit)  ((byte) & (1 << (bit)))
#define TOGGLE_BIT(byte, bit) ((byte) ^= (1 << (bit)))

// setup and main loop  
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);
  
  // Initialize hardware using pointers to structures
  initializeMotorPins(&motors);
  initializeSensorPins(&ultrasonic, &lineTracking);
  
  // Initialize servo
  myservo.attach(config.servoPin);
  myservo.write(90);
  delay(500);
  
  stopMotors(&motors);
  initializeBLE();
  
  Serial.println("=== TurtleBot with Advanced C Concepts ===");
  Serial.println("Using: Structs, Pointers, Enums, Bitwise Ops");
  Serial.println("===========================================");
}

//  MAIN LOOP 
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    if (!robotState.isConnected) {
      Serial.print("Connected to: ");
      Serial.println(central.address());
      robotState.isConnected = true;
      delay(100);
      safeBLEWrite("Connected - Manual Mode");
    }

    while (central.connected()) {
      // Process BLE commands
      if (rxCharacteristic.written()) {
        String command = rxCharacteristic.value();
        command.trim();
        if (command.length() > 0) {
          Serial.print("Received: ");
          Serial.println(command);
          handleBTCommand(command.charAt(0));
        }
      }

      // Run autonomous modes
      unsigned long currentTime = millis();
      if (currentTime - robotState.lastModeUpdate >= 100) {
        switch (robotState.currentMode) {
          case LINE_TRACKING:
            runLineTracking(&motors, &lineTracking);
            break;
          case OBSTACLE_AVOIDANCE:
            runObstacleAvoidance(&motors, &ultrasonic, &config);
            break;
          case MANUAL:
            // Manual mode - do nothing, wait for commands
            break;
        }
        robotState.lastModeUpdate = currentTime;
      }

      // Heartbeat every 5 seconds
      if (millis() - robotState.lastHeartbeat > 5000) {
        safeBLEWrite("HB");
        robotState.lastHeartbeat = millis();
      }

      BLE.poll();
      delay(1);
    }

    // Handle disconnection
    if (robotState.isConnected) {
      Serial.println("Disconnected");
      robotState.isConnected = false;
      stopMotors(&motors);
      robotState.currentMode = MANUAL;
    }
  }

  // Refresh advertising
  if (!robotState.isConnected && millis() - robotState.lastAdvertiseTime > 30000) {
    BLE.stopAdvertise();
    delay(100);
    BLE.advertise();
    robotState.lastAdvertiseTime = millis();
    Serial.println("Advertising refreshed");
  }
  
  delay(50);
}

//  INITIALIZATION FUNCTIONS 
void initializeMotorPins(MotorPins *motor) {
  pinMode(motor->ena, OUTPUT);
  pinMode(motor->in1, OUTPUT);
  pinMode(motor->in2, OUTPUT);
  pinMode(motor->enb, OUTPUT);
  pinMode(motor->in3, OUTPUT);
  pinMode(motor->in4, OUTPUT);
}

void initializeSensorPins(UltrasonicSensor *us, LineTrackingSensors *lt) {
  pinMode(us->trigPin, OUTPUT);
  pinMode(us->echoPin, INPUT);
  pinMode(lt->leftPin, INPUT);
  pinMode(lt->middlePin, INPUT);
  pinMode(lt->rightPin, INPUT);
}

void initializeBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE failed!");
    while (1) delay(1000);
  }

  BLE.setDeviceName("TurtleBot");
  BLE.setLocalName("TurtleBot");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(rxCharacteristic);
  uartService.addCharacteristic(txCharacteristic);
  BLE.addService(uartService);
  txCharacteristic.writeValue("TurtleBot Ready");
  BLE.advertise();
}

//  COMMAND HANDLER 
void handleBTCommand(char c) {
  if (c == '\r' || c == '\n' || c == '\0') return;
  if (c >= 'a' && c <= 'z') c -= 32; // Convert to uppercase

  safeBLEWrite((String("CMD: ") + c).c_str());
  delay(50);

  switch (c) {
    case 'F':
      if (robotState.currentMode == MANUAL) {
        setMotorDirection(&motors, FORWARD);
        robotState.currentDirection = FORWARD;
        safeBLEWrite("Moving Forward");
      }
      break;
      
    case 'B':
      if (robotState.currentMode == MANUAL) {
        setMotorDirection(&motors, BACKWARD);
        robotState.currentDirection = BACKWARD;
        safeBLEWrite("Moving Backward");
      }
      break;
      
    case 'L':
      if (robotState.currentMode == MANUAL) {
        setMotorDirection(&motors, LEFT);
        robotState.currentDirection = LEFT;
        safeBLEWrite("Turning Left");
      }
      break;
      
    case 'R':
      if (robotState.currentMode == MANUAL) {
        setMotorDirection(&motors, RIGHT);
        robotState.currentDirection = RIGHT;
        safeBLEWrite("Turning Right");
      }
      break;
      
    case 'S':
      robotState.currentMode = MANUAL;
      stopMotors(&motors);
      robotState.currentDirection = STOP;
      safeBLEWrite("Stopped - Manual Mode");
      break;
      
    case '1':
      robotState.currentMode = MANUAL;
      stopMotors(&motors);
      safeBLEWrite("Manual Mode Active");
      Serial.println("Mode: MANUAL");
      break;
      
    case '2':
      robotState.currentMode = OBSTACLE_AVOIDANCE;
      stopMotors(&motors);
      myservo.write(90);
      safeBLEWrite("Auto Mode - Obstacle Avoidance");
      Serial.println("Mode: OBSTACLE_AVOIDANCE");
      break;

    case '3':
      robotState.currentMode = LINE_TRACKING;
      stopMotors(&motors);
      safeBLEWrite("Line Tracking Mode Active");
      Serial.println("Mode: LINE_TRACKING");
      break;
      
    default:
      safeBLEWrite("Unknown Command");
      break;
  }
}

// motor control code for moving robot with bitwise operators
void setMotorDirection(MotorPins *motor, Direction dir) {
  // set direction for motors to move 
  uint8_t motorControl = 0x00;
  
  switch (dir) {
    case FORWARD:
      SET_BIT(motorControl, 0); // IN1 = HIGH
      CLEAR_BIT(motorControl, 1); // IN2 = LOW
      SET_BIT(motorControl, 2); // IN3 = HIGH
      CLEAR_BIT(motorControl, 3); // IN4 = LOW
      
      digitalWrite(motor->in1, CHECK_BIT(motorControl, 0) ? HIGH : LOW);
      digitalWrite(motor->in2, CHECK_BIT(motorControl, 1) ? HIGH : LOW);
      digitalWrite(motor->in3, CHECK_BIT(motorControl, 2) ? HIGH : LOW);
      digitalWrite(motor->in4, CHECK_BIT(motorControl, 3) ? HIGH : LOW);
      break;
      
    case BACKWARD:
      CLEAR_BIT(motorControl, 0); // IN1 = LOW
      SET_BIT(motorControl, 1); // IN2 = HIGH
      CLEAR_BIT(motorControl, 2); // IN3 = LOW
      SET_BIT(motorControl, 3); // IN4 = HIGH
      
      digitalWrite(motor->in1, CHECK_BIT(motorControl, 0) ? HIGH : LOW);
      digitalWrite(motor->in2, CHECK_BIT(motorControl, 1) ? HIGH : LOW);
      digitalWrite(motor->in3, CHECK_BIT(motorControl, 2) ? HIGH : LOW);
      digitalWrite(motor->in4, CHECK_BIT(motorControl, 3) ? HIGH : LOW);
      break;
      
    case LEFT:
      SET_BIT(motorControl, 0); // IN1 = HIGH
      CLEAR_BIT(motorControl, 1); // IN2 = LOW
      CLEAR_BIT(motorControl, 2); // IN3 = LOW
      SET_BIT(motorControl, 3); // IN4 = HIGH
      
      digitalWrite(motor->in1, CHECK_BIT(motorControl, 0) ? HIGH : LOW);
      digitalWrite(motor->in2, CHECK_BIT(motorControl, 1) ? HIGH : LOW);
      digitalWrite(motor->in3, CHECK_BIT(motorControl, 2) ? HIGH : LOW);
      digitalWrite(motor->in4, CHECK_BIT(motorControl, 3) ? HIGH : LOW);
      break;
      
    case RIGHT:
      CLEAR_BIT(motorControl, 0); // IN1 = LOW
      SET_BIT(motorControl, 1); // IN2 = HIGH
      SET_BIT(motorControl, 2); // IN3 = HIGH
      CLEAR_BIT(motorControl, 3); // IN4 = LOW
      
      digitalWrite(motor->in1, CHECK_BIT(motorControl, 0) ? HIGH : LOW);
      digitalWrite(motor->in2, CHECK_BIT(motorControl, 1) ? HIGH : LOW);
      digitalWrite(motor->in3, CHECK_BIT(motorControl, 2) ? HIGH : LOW);
      digitalWrite(motor->in4, CHECK_BIT(motorControl, 3) ? HIGH : LOW);
      break;
      
    case STOP:
      stopMotors(motor);
      return;
  }
  
  analogWrite(motor->ena, motor->speed);
  analogWrite(motor->enb, motor->speed);
}

void stopMotors(MotorPins *motor) {
  digitalWrite(motor->in1, LOW);
  digitalWrite(motor->in2, LOW);
  digitalWrite(motor->in3, LOW);
  digitalWrite(motor->in4, LOW);
  analogWrite(motor->ena, 0);
  analogWrite(motor->enb, 0);
}

// code for distance sensor (ultrasonic) 
long getDistance(UltrasonicSensor *sensor) {
  // get distance value from ultrasonic sensor
  digitalWrite(sensor->trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sensor->trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->trigPin, LOW);
  
  long duration = pulseIn(sensor->echoPin, HIGH, 25000);
  long distance = (duration * 0.034) / 2;
  
  if (duration == 0 || distance <= 0 || distance > 300) {
    distance = 300;
  }
  
  sensor->lastDistance = distance;
  return distance;
}

// line sensor code for line tracing mode
uint8_t readLineTrackingSensors(LineTrackingSensors *lt) {
  lt->sensorState = 0x00; // Clear all bits
  
  // Read sensors with simple debouncing
  uint8_t reading1 = 0x00;
  uint8_t reading2 = 0x00;
  
  // First reading
  if (digitalRead(lt->leftPin) == HIGH) SET_BIT(reading1, 0);
  if (digitalRead(lt->middlePin) == HIGH) SET_BIT(reading1, 1);
  if (digitalRead(lt->rightPin) == HIGH) SET_BIT(reading1, 2);
  
  delayMicroseconds(100); // Tiny delay for debounce
  
  // Second reading
  if (digitalRead(lt->leftPin) == HIGH) SET_BIT(reading2, 0);
  if (digitalRead(lt->middlePin) == HIGH) SET_BIT(reading2, 1);
  if (digitalRead(lt->rightPin) == HIGH) SET_BIT(reading2, 2);
  
  // Only accept reading if both match (debouncing)
  lt->sensorState = reading1 & reading2;
  
  return lt->sensorState;
}

void runLineTracking(MotorPins *motor, LineTrackingSensors *lt) {
  uint8_t sensors = readLineTrackingSensors(lt);
  
  // Skip processing if state hasn't changed (efficiency optimization)
  if (sensors == lt->lastState) {
    return;
  }
  lt->lastState = sensors;
  
  // Use bitwise operations to check sensor states
  bool leftDetected = CHECK_BIT(sensors, 0);
  bool middleDetected = CHECK_BIT(sensors, 1);
  bool rightDetected = CHECK_BIT(sensors, 2);
  
  // Motor control byte for efficient direction setting
  uint8_t motorControl = 0x00;
  
  if (middleDetected) {
    // On line - move forward at reduced speed
    SET_BIT(motorControl, 0); // IN1 = HIGH
    CLEAR_BIT(motorControl, 1); // IN2 = LOW
    SET_BIT(motorControl, 2); // IN3 = HIGH
    CLEAR_BIT(motorControl, 3); // IN4 = LOW
    
    digitalWrite(motor->in1, HIGH);
    digitalWrite(motor->in2, LOW);
    digitalWrite(motor->in3, HIGH);
    digitalWrite(motor->in4, LOW);
    analogWrite(motor->ena, lt->lineSpeed); // Use reduced speed
    analogWrite(motor->enb, lt->lineSpeed);
    
    // Throttled BLE message - only send every 2 seconds
    if (millis() % 2000 < 50) {
      safeBLEWrite("Line: Following");
    }
    
  } else if (leftDetected && !rightDetected) {
    // Line to left - turn at reduced speed
    uint8_t turnSpeed = lt->lineSpeed * 0.8; // Even slower for turns
    
    digitalWrite(motor->in1, HIGH);
    digitalWrite(motor->in2, LOW);
    digitalWrite(motor->in3, LOW);
    digitalWrite(motor->in4, HIGH);
    analogWrite(motor->ena, turnSpeed);
    analogWrite(motor->enb, turnSpeed);
    
    safeBLEWrite("Line: Left");
    
  } else if (rightDetected && !leftDetected) {
    // Line to right - turn at reduced speed
    uint8_t turnSpeed = lt->lineSpeed * 0.8; // Even slower for turns
    
    digitalWrite(motor->in1, LOW);
    digitalWrite(motor->in2, HIGH);
    digitalWrite(motor->in3, HIGH);
    digitalWrite(motor->in4, LOW);
    analogWrite(motor->ena, turnSpeed);
    analogWrite(motor->enb, turnSpeed);
    
    safeBLEWrite("Line: Right");
    
  } else if (sensors == 0x00) {
    // All sensors LOW (no line detected) - stop
    stopMotors(motor);
    safeBLEWrite("Line: Lost");
    delay(100); // Brief pause when line is lost
  }
}

//  OBSTACLE AVOIDANCE 
void runObstacleAvoidance(MotorPins *motor, UltrasonicSensor *sensor, RobotConfig *cfg) {
  myservo.write(90);
  delay(100);
  
  long frontDistance = getDistance(sensor);
  Serial.println(frontDistance);
  
  if (frontDistance > cfg->obstacleDistance) {
    setMotorDirection(motor, FORWARD);
    if (millis() % 1000 < 100) {
      safeBLEWrite("Auto: Moving Forward");
    }
  } else {
    stopMotors(motor);
    safeBLEWrite("Obstacle Detected - Scanning");
    delay(200);
    
    // Scan right
    myservo.write(30);
    delay(cfg->scanDelay);
    long rightDistance = getDistance(sensor);
    
    // Scan left
    myservo.write(150);
    delay(cfg->scanDelay);
    long leftDistance = getDistance(sensor);
    
    // Return to center
    myservo.write(90);
    delay(cfg->scanDelay);
    
    // Decision making algo
    if (rightDistance > cfg->obstacleDistance && leftDistance > cfg->obstacleDistance) {
      if (rightDistance >= leftDistance) {
        safeBLEWrite("Auto: Turning Right");
        setMotorDirection(motor, RIGHT);
        delay(cfg->turnDelay);
      } else {
        safeBLEWrite("Auto: Turning Left");
        setMotorDirection(motor, LEFT);
        delay(cfg->turnDelay);
      }
    } else if (rightDistance > cfg->obstacleDistance) {
      safeBLEWrite("Auto: Turning Right");
      setMotorDirection(motor, RIGHT);
      delay(cfg->turnDelay);
    } else if (leftDistance > cfg->obstacleDistance) {
      safeBLEWrite("Auto: Turning Left");
      setMotorDirection(motor, LEFT);
      delay(cfg->turnDelay);
    } else {
      safeBLEWrite("Auto: Blocked - Reversing");
      setMotorDirection(motor, BACKWARD);
      delay(600);
      setMotorDirection(motor, RIGHT);
      delay(800);
    }
    
    stopMotors(motor);
    delay(100);
  }
}

//  BLE HELPER 
bool safeBLEWrite(const char* message) {
  if (robotState.isConnected) {
    txCharacteristic.writeValue(message);
    return true;
  }
  return false;
}
