/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Arduino Smart Car - SIMPLIFIED 90° SCANNING
 * Uses ONLY 0°, 90°, and 180° - no extra angles needed
 *   
 * 
 */

#include <Servo.h>
// Motor pins
#define speedPinR 3
#define RightDirectPin1  12
#define RightDirectPin2  11
#define speedPinL 6
#define LeftDirectPin1  7
#define LeftDirectPin2  8
// Servo and ultrasonic pins
#define SERVO_PIN     9
#define Echo_PIN      2
#define Trig_PIN      10
// SIMPLIFIED: Only 3 essential angles
#define LOOK_RIGHT     0    // FULLY RIGHT (perpendicular to travel)
#define LOOK_FORWARD  90    // Straight ahead  
#define LOOK_LEFT    180    // FULLY LEFT (perpendicular to travel)
// Algorithm parameters
#define TARGET_DISTANCE     15    // Desired distance from wall
#define MIN_FRONT_CLEAR     25    // Minimum clear space ahead
#define WALL_TOO_FAR        50    // Distance where we consider wall lost
// Speed parameters
#define BASE_SPEED          130
#define TURN_GAIN           4     // How aggressively to turn
#define MIN_SPEED           80
#define MAX_SPEED           200
Servo head;
// ==================== MOTOR CONTROL ====================
void go_Advance(void) {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void set_Motorspeed(int speed_L, int speed_R) {
  speed_L = constrain(speed_L, 0, 255);
  speed_R = constrain(speed_R, 0, 255);
  analogWrite(speedPinL, speed_L); 
  analogWrite(speedPinR, speed_R);   
}
void stop_Stop() {
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
  set_Motorspeed(0, 0);
}
// ==================== END MOTOR CONTROL ====================
// ==================== ULTRASONIC ====================
int getDistance() {
  long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH, 30000);
  if (echo_distance <= 0) {
    return 200;
  }
  return echo_distance * 0.034 / 2;
}
int getDistanceAtAngle(int angle) {
  head.write(angle);
  delay(150); // Give servo time to reach position
  int distance = getDistance();
  delay(30);
  return distance;
}
// ==================== END ULTRASONIC ====================
// ==================== SIMPLIFIED WALL FOLLOWING ====================
void simpleWallFollow() {
  static bool followingRight = true;
  static int consecutiveNoWall = 0;
  // STEP 1: Check the wall we're supposed to be following
  int sideDist;
  if (followingRight) {
    sideDist = getDistanceAtAngle(LOOK_RIGHT); // 0° - FULLY RIGHT
  } else {
    sideDist = getDistanceAtAngle(LOOK_LEFT);  // 180° - FULLY LEFT
  }
  // STEP 2: Check directly ahead for obstacles
  int frontDist = getDistanceAtAngle(LOOK_FORWARD); // 90° - STRAIGHT
  // STEP 3: Return servo to forward position
  head.write(LOOK_FORWARD);
  // Debug output
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print("cm | ");
  if (followingRight) {
    Serial.print("Right (0°): ");
  } else {
    Serial.print("Left (180°): ");
  }
  Serial.print(sideDist);
  Serial.println("cm");
  // ===== DECISION LOGIC =====
  // CASE 1: OBSTACLE DIRECTLY AHEAD
  if (frontDist < MIN_FRONT_CLEAR && frontDist > 5) {
    Serial.println("OBSTACLE AHEAD!");    
    if (followingRight) {
      // Following right wall, obstacle ahead = turn LEFT (away from wall)
      Serial.println("Turning LEFT around corner");
      // Sharp left turn: left wheel slow, right wheel fast
      int leftSpeed = constrain(BASE_SPEED - 80, MIN_SPEED, MAX_SPEED);
      int rightSpeed = constrain(BASE_SPEED + 80, MIN_SPEED, MAX_SPEED);   
      go_Advance();
      set_Motorspeed(leftSpeed, rightSpeed);
      
      // Turn for enough time to go around a 90° corner
      delay(400);
      
    } else {
      // Following left wall, obstacle ahead = turn RIGHT (away from wall)
      Serial.println("Turning RIGHT around corner");
      
      // Sharp right turn: left wheel fast, right wheel slow
      int leftSpeed = constrain(BASE_SPEED + 80, MIN_SPEED, MAX_SPEED);
      int rightSpeed = constrain(BASE_SPEED - 80, MIN_SPEED, MAX_SPEED);
      
      go_Advance();
      set_Motorspeed(leftSpeed, rightSpeed);
      
      delay(400);
    }
    
    consecutiveNoWall = 0;
    return;
  }
  
  // CASE 2: NORMAL WALL FOLLOWING
  if (sideDist > 5 && sideDist < WALL_TOO_FAR) {
    consecutiveNoWall = 0; // Reset counter
    // Calculate error from target distance
    int error = sideDist - TARGET_DISTANCE;
    // Calculate adjustment (proportional control)
    int adjustment = error * TURN_GAIN;
    adjustment = constrain(adjustment, -60, 60);
    // Calculate motor speeds
    int leftSpeed, rightSpeed;
    if (followingRight) {
      if (error > 0) { 
        // Too far from right wall: Turn RIGHT
        // Right turn = left wheel faster, right wheel slower
        leftSpeed = BASE_SPEED + adjustment;
        rightSpeed = BASE_SPEED - adjustment;
        Serial.print("Too far (");
        Serial.print(sideDist);
        Serial.println("cm) - turning RIGHT");
      } else { 
        // Too close to right wall: Turn LEFT
        // Left turn = left wheel slower, right wheel faster
        leftSpeed = BASE_SPEED + adjustment; // adjustment is negative
        rightSpeed = BASE_SPEED - adjustment; // subtract negative = add
        Serial.print("Too close (");
        Serial.print(sideDist);
        Serial.println("cm) - turning LEFT");
      }
    } else {
      if (error > 0) { 
        // Too far from left wall: Turn LEFT
        // Left turn = left wheel slower, right wheel faster
        leftSpeed = BASE_SPEED - adjustment;
        rightSpeed = BASE_SPEED + adjustment;
        Serial.print("Too far (");
        Serial.print(sideDist);
        Serial.println("cm) - turning LEFT");
      } else { 
        // Too close to left wall: Turn RIGHT
        // Right turn = left wheel faster, right wheel slower
        leftSpeed = BASE_SPEED - adjustment; // adjustment negative
        rightSpeed = BASE_SPEED + adjustment; // add negative = subtract
        Serial.print("Too close (");
        Serial.print(sideDist);
        Serial.println("cm) - turning RIGHT");
      }
    }
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    // Apply
    go_Advance();
    set_Motorspeed(leftSpeed, rightSpeed);
  } else {
    // CASE 3: WALL NOT DETECTED
    consecutiveNoWall++;
    Serial.print("No wall detected (");
    Serial.print(consecutiveNoWall);
    Serial.println(")");
    if (consecutiveNoWall > 5) {
      // Been too long without a wall - search for it
      Serial.println("Searching for wall...");
      if (followingRight) {
        // Turn right to find right wall
        go_Advance();
        set_Motorspeed(BASE_SPEED + 50, BASE_SPEED - 50);
      } else {
        // Turn left to find left wall
        go_Advance();
        set_Motorspeed(BASE_SPEED - 50, BASE_SPEED + 50);
      }
      delay(200);
      consecutiveNoWall = 0;
    } else {
      // Go straight slowly
      go_Advance();
      set_Motorspeed(BASE_SPEED - 30, BASE_SPEED - 30);
    }
  }
  delay(100);
}
// ==================== EVEN SIMPLER VERSION ====================
void ultraSimpleFollow() {
  // Always follow right wall (simplest approach)
  
  // Look fully right
  int rightDist = getDistanceAtAngle(LOOK_RIGHT);
  
  // Look forward
  int frontDist = getDistanceAtAngle(LOOK_FORWARD);
  
  head.write(LOOK_FORWARD);
  
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print("cm | Right: ");
  Serial.print(rightDist);
  Serial.println("cm");
  
  // Simple logic
  if (frontDist < 20 && frontDist > 5) {
    // Obstacle ahead - turn left
    Serial.println("Turning LEFT");
    go_Advance();
    set_Motorspeed(100, 180);
    delay(400);
  } 
  else if (rightDist > 5 && rightDist < 100) {
    // Wall detected - adjust distance
    int leftSpeed = BASE_SPEED;
    int rightSpeed = BASE_SPEED;
    
    if (rightDist > TARGET_DISTANCE + 5) {
      // Too far - turn right
      leftSpeed = BASE_SPEED + 40;
      rightSpeed = BASE_SPEED - 20;
      Serial.println("Too far - turning RIGHT");
    } 
    else if (rightDist < TARGET_DISTANCE - 5) {
      // Too close - turn left
      leftSpeed = BASE_SPEED - 20;
      rightSpeed = BASE_SPEED + 40;
      Serial.println("Too close - turning LEFT");
    }
    else {
      Serial.println("Good distance");
    }
    
    go_Advance();
    set_Motorspeed(leftSpeed, rightSpeed);
  }
  else {
    // No wall - go straight
    Serial.println("No wall - turning RIGHT to search");
    go_Advance();
    set_Motorspeed(BASE_SPEED+40, BASE_SPEED-20);
  }
  
  delay(150);
}
// ==================== SETUP ====================
void setup() {
  // Setup motor pins
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();  
  // Setup ultrasonic
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN, INPUT); 
  digitalWrite(Trig_PIN, LOW);  
  // Setup servo
  head.attach(SERVO_PIN);   
  // Test servo can reach all positions
  Serial.begin(9600);
  Serial.println("Testing servo range...");  
  head.write(LOOK_RIGHT);    // 0°
  delay(1000);
  Serial.println("Servo at 0° (FULL RIGHT)");
  
  head.write(LOOK_FORWARD);  // 90°
  delay(1000);
  Serial.println("Servo at 90° (FORWARD)");
  
  head.write(LOOK_LEFT);     // 180°
  delay(1000);
  Serial.println("Servo at 180° (FULL LEFT)");
  
  head.write(LOOK_FORWARD);  // Return to forward
  delay(1000);
  
  Serial.println("==================================");
  Serial.println("SIMPLIFIED 90° WALL FOLLOWING");
  Serial.println("Only uses 3 angles: 0°, 90°, 180°");
  Serial.println("Following RIGHT wall by default");
  Serial.println("Target distance: " + String(TARGET_DISTANCE) + "cm");
  Serial.println("==================================");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Use the simple wall following
  // simpleWallFollow();
  
  // Or try the ultra-simple version if above doesn't work:
  ultraSimpleFollow();
}