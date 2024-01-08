// JJROBOTS ROBOTIC ARM. SCARA ROBOT.
// Author: Jose Julio & Juan pedro & Jonathan (JJROBOTS)
// Hardware: Arduino Mega 2560 & ESP8266
// Date: 5/01/2024
// Last updated: 20/09/2019
// Version: 0.16
// Project page : http://jjrobots.com/
// License: Open Software GPL License v2

// Hardware: Arduino Mega 2560
// Board:Arduino Mega 2560

// Support for laser range sensor VL53L0X on I2C port
// You need to install the library: VL53L0X by Pololu

// Motor1:
// Enable: Arduino pin D23 (PA1)
// Step: Arduino pin 24 (PA2)
// Dir: Arduino pin 25 (PA3)

// Motor2:
// Enable: Arduino pin D23 (PA1)
// Step: Arduino pin 26 (PA4)
// Dir: Arduino pin 27 (PA5)

// Motor3:
// Enable: Arduino pin D23 (PA1)
// Step: Arduino pin 28 (PA6)
// Dir: Arduino pin 29 (PA7)

// Mircrostepping : Arduino pin A4 (PA05) (default 1/16)

#define VERSION "Scara v0.16"
//#define DEBUG 0

// ROBOT and USER configuration parameters
#include "Configuration.h"
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

Servo servo1; // Wrist orientation
Servo servo2; // Gripper open/close

VL53L0X sensor; // Range sensor

// Configuration: Pins, servos, Steppers, Wifi...
void setup()
{
  // STEPPER PINS ON JJROBOTS DEVIA M0 BOARD
  pinMode(23, OUTPUT); // ENABLE MOTORS   ATSAMD21:PA16
  pinMode(24, OUTPUT); // STEP MOTOR 1 ATSAMD21:PA15
  pinMode(25, OUTPUT); // DIR MOTOR 1  ATSAMD21:PA20
  pinMode(26, OUTPUT); // STEP MOTOR 2 ATSAMD21:PA21
  pinMode(27, OUTPUT); // DIR MOTOR 2  ATSAMD21:PA06
  pinMode(28, OUTPUT); // STEP MOTOR 3 ATSAMD21:PA07
  pinMode(29, OUTPUT); // DIR MOTOR 3  ATSAMD21:PA18

  pinMode(A4, OUTPUT);    // Microstepping output
  digitalWrite(A4, HIGH); // 1/16 (default config)

  // Servos
  pinMode(3, OUTPUT); // SERVO1  
  pinMode(4, OUTPUT); // SERVO2  

  pinMode(12, OUTPUT); // Electromagnet output (PA19) [optional]
  digitalWrite(12, LOW); //Disabled

  pinMode(RED_LED, OUTPUT); // RED LED
  pinMode(GREEN_LED, OUTPUT); // GREEN LED
  pinMode(SWITCH_IN, INPUT_PULLUP);  // Input Switch
  digitalWrite(SWITCH_IN, OUTPUT); // PULLUP

  digitalWrite(23, HIGH);  // Disbale stepper motors
  digitalWrite(RED_LED, HIGH);  // RED LED ON

  // Serial ports initialization
  delay(100);
  Serial.begin(115200); // Serial output to console
  Serial1.begin(115200); // Wifi initialization
  Wire.begin();
  delay(1000);

#ifdef DEBUG
  delay(10000);  // Only needed for serial debug
  Serial.println(VERSION);
#endif

  // WIFI MODULE INITIALIZATION PROCESS
  Serial.println("WIFI init");
  Serial1.flush();
  Serial1.print("+++");  // To ensure we exit the transparent transmision mode
  delay(100);

  ESPsendCommand(String("AT"), String("OK"), 1);
  //ESPsendCommand(String("AT+RST"), String("OK"), 2); // ESP Wifi module RESET
  //ESPwait(String("ready"), 6);
  ESPsendCommand(String("AT+GMR"), String("OK"), 5);

  Serial1.println("AT+CIPSTAMAC?");
  ESPgetMac();
  Serial.print("MAC:");
  Serial.println(MAC);
  delay(100);
  ESPsendCommand(String("AT+CWMODE=2"), String("OK"), 3); // Soft AP mode
  //SerialUSB.println("Aqui tambien");
  // Generate Soft AP. SSID=JJROBOTS_XX (XX= user MAC last characters), PASS=87654321
  String cmd =  String("AT+CWSAP=\"JJROBOTS_") + MAC.substring(MAC.length()-2,MAC.length()) + String("\",\"87654321\",5,3");
  ESPsendCommand(cmd, String("OK"), 6);

  // Start UDP SERVER on port 2222, telemetry port 2223
  Serial.println("Start UDP server");
  ESPsendCommand(String("AT+CIPMUX=0"), String("OK"), 3);  // Single connection mode
  delay(10);
  ESPsendCommand(String("AT+CIPMODE=1"), String("OK"), 3); // Transparent mode
  delay(10);
  String Telemetry = String("AT+CIPSTART=\"UDP\",\"") + String(TELEMETRY) + String("\",2223,2222,0");
  ESPsendCommand(Telemetry, String("CONNECT"), 3);
  Serial.println(Telemetry);
  delay(200);
  ESPsendCommand(String("AT+CIPSEND"), String('>'), 2); // Start transmission (transparent mode)

  // Start TCP SERVER on port 2222, telemetry port 2223
  //SerialUSB.println("Start TCP server");
  //ESPsendCommand("AT+CIPCLOSE=0","OK",3);
  //ESPsendCommand("AT+CIPSERVER=0","OK",3);
  //ESPsendCommand("AT+CIPMUX=1", "OK", 3);  // Multiple connection mode
  //ESPsendCommand("AT+CIPMODE=1", "OK", 3); // Transparent mode
  //ESPsendCommand("AT+CIPSERVER=1,2222", "OK", 3); // TCP server
  delay(100);

  // Init servos
  initServo();
  moveServo1(SERVO1_NEUTRAL);
  moveServo2(SERVO2_NEUTRAL);

  // Debug: Output parameters
  //Serial.print("Max_acceleration_x: ");
  //Serial.println(acceleration_x);
  //Serial.print("Max_acceleration_y: ");
  //Serial.println(acceleration_y);
  //Serial.print("Max speed X: ");
  //Serial.println(MAX_SPEED_X);
  //Serial.print("Max speed Y: ");
  //Serial.println(MAX_SPEED_Y);

  // STEPPER MOTORS INITIALIZATION
  Serial.println("Steper motors initialization...");
  timersConfigure();
  Serial.println("Timers initialized");
  timersStart(); //starts the timers
  Serial.println("Timers started");
  delay(100);
  Serial.println("Moving to initial position...");

  configSpeed(MAX_SPEED_M1 / 10, MAX_SPEED_M2 / 10, MAX_SPEED_M3);
  configAcceleration(MAX_ACCEL_M1 / 2, MAX_ACCEL_M2 / 2, MAX_ACCEL_M3);
  setSpeedAcc();

  //Initializing init position
  target_angleA1 = ROBOT_INITIAL_POSITION_M1;
  target_angleA2 = ROBOT_INITIAL_POSITION_M2 + ROBOT_INITIAL_POSITION_M1 * AXIS2_AXIS1_correction;
  position_M1 = target_angleA1 * M1_AXIS_STEPS_PER_UNIT;
  position_M2 = target_angleA2 * M2_AXIS_STEPS_PER_UNIT;
  position_M3 = ROBOT_INITIAL_POSITION_M3 * M3_AXIS_STEPS_PER_UNIT;

#ifdef INITIALIZE_TO_MAXIMUNS
  motorsCalibration();
#endif

  //target_angleA1 = ROBOT_INITIAL_POSITION_M1;
  //target_angleA2 = ROBOT_INITIAL_POSITION_M2 + ROBOT_INITIAL_POSITION_M1*AXIS2_AXIS1_correction;

  configSpeed(MAX_SPEED_M1, MAX_SPEED_M2, MAX_SPEED_M3);
  configAcceleration(MAX_ACCEL_M1, MAX_ACCEL_M2, MAX_ACCEL_M3);
  setSpeedAcc();
  target_position_M1 = position_M1;
  target_position_M2 = position_M2;
  target_position_M3 = position_M3;

  Serial.println("Initial position configured!");
  Serial.println(ROBOT_ARM1_LENGTH);
  Serial.println(ROBOT_ARM2_LENGTH);
  Serial.println(ROBOT_ARM1_LENGTH + ROBOT_ARM2_LENGTH);

  Serial.println("Laser range sensor initialization...");
  sensor.init();
  sensor.startContinuous(45); // Setup as continuos timed mode (45ms)

  Serial.println(" Ready...");
  Serial.print(" JJROBOTS SCARA ");
  Serial.println(VERSION);
  timer_old = micros();
  slow_timer_old = millis();
  laser_timer_old = millis();
  timeout_counter = 0;

  digitalWrite(RED_LED, LOW);  // RED LED OFF
  digitalWrite(GREEN_LED, HIGH);  // GREEN LED ON

  // Enable motors
  digitalWrite(23, LOW);  // Enable motors
}

// *************** APLICATION MAIN LOOP ***********************
void loop()
{
  MsgRead();     // Read network messages
  USBMsgRead();  // Read USB messages
  if (newMessage)
  {
    newMessage = 0;
    //debugMsg();
    if (mode == 1) {
      // Manual mode: Direct kinematics
      if ((iCH1 != NODATA) || (iCH2 != NODATA)) {
        setAxis_1_2(iCH1 / 100.0, iCH2 / 100.0);
        setSpeedAcc();
        trajectory_motor_speed_adjust();  // Function to sync both motors
      }
      //if (iCH2 != NODATA)
      //  setAxis_1_2(iCH1/100.0,iCH2/100.0);
      if (iCH3 != NODATA)
        setAxis3(iCH3 / 100.0);

      // Servos:
      if (iCH4 != NODATA)
        moveServo1(iCH4 * SERVO1_RANGE / 1000.0 + SERVO1_MIN_PULSEWIDTH);
      if (iCH5 != NODATA)
        moveServo2(iCH5 * SERVO2_RANGE / 1000.0 + SERVO2_MIN_PULSEWIDTH);
    }
    else if (mode == 2) {
      // Inverse Kinematic mode
      if (iCH6 == 1) {
        elbow = 1;
        //Serial.println("->Elbow:1");
      }
      else
        elbow = 0;
      float x = iCH1 / 10.0;
      float y = iCH2 / 10.0;
      float A1, A2;

      Serial.print("->IK time:");
      long t0 = micros();
      InverseKinematic(x, y, ROBOT_ARM1_LENGTH, ROBOT_ARM2_LENGTH, elbow, &A1, &A2);
      long t1 = micros();
      Serial.println(t1 - t0); // First implementation was 560us
      Serial.print("->IK:");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.print(",");
      Serial.print(iCH3);
      Serial.print(" :");
      Serial.print(A1);
      Serial.print(",");
      Serial.println(A2);
      setAxis_1_2(A1, A2);
      if (iCH3 != NODATA)
        target_position_M3 = (iCH3 / 100.0) * M3_AXIS_STEPS_PER_UNIT;
      // Servos:
      if (iCH4 != NODATA)
        moveServo1(iCH4 * SERVO1_RANGE / 1000.0 + SERVO1_MIN_PULSEWIDTH);
      if (iCH5 != NODATA)
        moveServo2(iCH5 * SERVO2_RANGE / 1000.0 + SERVO2_MIN_PULSEWIDTH);

      setSpeedAcc();
      trajectory_motor_speed_adjust();  // Function to sync both motors
    }
    else if (mode == 3) {
      // Trajectory mode
      //Serial.print("->T MODE:");
      //Serial.println(trajectory_num_points);
    }

    else if (mode == 4) { // Emergency stop
      // Stop the robot in the actual position
      //float a1 = (position_M1 / M1_AXIS_STEPS_PER_UNIT);
      //float a2 = (position_M2 / M2_AXIS_STEPS_PER_UNIT) - a1 * AXIS2_AXIS1_correction;
      //float az =  position_M3 / M3_AXIS_STEPS_PER_UNIT;
      //setAxis_1_2(a1, a2);
      //setAxis3(az);
      // Disable motors
      digitalWrite(11, HIGH);  // Disable motors
    }

    else if (mode == 5) { // Robot calibration
      Serial.println("->Motors calibration...");
      motorsCalibration();
      setSpeedAcc();
      working = false;
      trajectory_mode = false;
    }
  }

  timer_value = micros();
  dt = timer_value - timer_old;
  if (dt >= 1000) { // 1Khz loop for position,speed and acceleration control
    if (dt > 1500) {
      Serial.print("!!");  // Timing warning
      Serial.println(dt);
    }
    timer_old = timer_value;

    positionControl(1000);   // position, speed and acceleration control of stepper motors
    // Trajectory mode manage...
    if (trajectory_mode) {
      trajectory_processing = false; // end of trajectory processing...
      working = true;  // Work to do...
      // Are we reaching next point without total stop? => Go to next point
      //if ((M1stopping || (dir_M1 == 0)) && (M2stopping || (dir_M2 == 0)) && (M3stopping || (dir_M3 == 0))) {
      diff_M1 = myAbs(target_position_M1 - position_M1);
      diff_M2 = myAbs(target_position_M2 - position_M2);
      diff_M3 = myAbs(target_position_M3 - position_M3);
      if ((diff_M1 < trajectory_tolerance_M1) && (diff_M2 < trajectory_tolerance_M2) && (diff_M3 < trajectory_tolerance_M3)) {
        // Go to next point
        if (trajectory_point <= trajectory_num_points) {
          Serial.print("->T ");
          Serial.println(trajectory_point);
          //Serial.print(" ");
          //Serial.print(trajectory_vector[trajectory_point][0]);
          //Serial.print(" ");
          //Serial.println(trajectory_vector[trajectory_point][1]);
          setAxis_1_2(trajectory_vector[trajectory_point][0], trajectory_vector[trajectory_point][1]);
          setSpeedAcc();
          trajectory_motor_speed_adjust();  // Function to sync both motors
          if (trajectory_vector[trajectory_point][2] != NODATA)
            target_position_M3 = trajectory_vector[trajectory_point][2] * M3_AXIS_STEPS_PER_UNIT;
          if (trajectory_vector[trajectory_point][3] != NODATA)
            moveServo1(trajectory_vector[trajectory_point][3] * SERVO1_RANGE / 1000.0 + SERVO1_MIN_PULSEWIDTH);
          if (trajectory_vector[trajectory_point][4] != NODATA)
            moveServo2(trajectory_vector[trajectory_point][4] * SERVO2_RANGE / 1000.0 + SERVO2_MIN_PULSEWIDTH);
          trajectory_point++;
        }
        else {
          trajectory_mode = false;
          trajectory_point = 0;
          trajectory_processing = false;
        }
      }
    }

    loop_counter += 1;

    // Read laser range sensor every 50ms
    laser_timer_value = millis();
    if ((laser_timer_value - laser_timer_old) >= 48) {
      laser_timer_old = laser_timer_value;
      digitalWrite(RED_LED, HIGH);  
      actual_distance = sensor.readRangeContinuousMillimeters();  // Read laser range sensor...
      digitalWrite(RED_LED, LOW); 
      if (actual_distance > 999)
        actual_distance = 999;
    }

    // Debug loop counter
    if (loop_counter % 10 == 0) {
      char message[80];
      //sprintf(message, "#%d:%d,%d,%d,%d", loop_counter/10,actual_angleA1, actual_angleA2,speed_M1,speed_M2);
      //Serial.println(message);
    }

    slow_timer_value = millis();
    if ((slow_timer_value - slow_timer_old) >= 50) {  // Slow loop (20hz)
      char message[80];

      slow_timer_old = slow_timer_value;

      if (trajectory_mode)
        working = true;
      else{
        // Check if robot is stopped (reach final position)
        diff_M1 = myAbs(target_position_M1 - position_M1);
        diff_M2 = myAbs(target_position_M2 - position_M2);
        diff_M3 = myAbs(target_position_M3 - position_M3);
        if ((diff_M1 < STOP_TOLERANCE) && (diff_M2 < STOP_TOLERANCE) && (diff_M3 < STOP_TOLERANCE))
          working = false;
        else
          working = true;
      }
      
      // Timestamp for status message...
      timestamp += 1;
      if (timestamp > 999)
        timestamp = 0;

      // Calculate actual robot angles based on internal motor positions (steps)
      actual_angleA1 = (position_M1 / M1_AXIS_STEPS_PER_UNIT) * 10;
      actual_angleA2 = (position_M2 / M2_AXIS_STEPS_PER_UNIT) * 10 - actual_angleA1 * AXIS2_AXIS1_correction;
      actual_valueZ = (position_M3 / M3_AXIS_STEPS_PER_UNIT) * 10;

      //uint16_t debug_value = acceleration_M2;
      if (trajectory_processing)
        sprintf(message, "$$2,%d,%d,%d,%d,%d,%d,%d", actual_angleA1, actual_angleA2, actual_valueZ, actual_valueG / 10, actual_valueW / 10, actual_distance, timestamp);
      else if (working)
        sprintf(message, "$$1,%d,%d,%d,%d,%d,%d,%d", actual_angleA1, actual_angleA2, actual_valueZ, actual_valueG / 10, actual_valueW / 10, actual_distance, timestamp);
      else
        sprintf(message, "$$0,%d,%d,%d,%d,%d,%d,%d", actual_angleA1, actual_angleA2, actual_valueZ, actual_valueG / 10, actual_valueW / 10, actual_distance, timestamp);

      Serial.println(message);   
      if (enable_udp_output) {       // Output UDP messages if we detect an UDP external interface
        Serial1.println(message);  
      }  
    } // 20hz loop
  } // 1Khz loop
}
