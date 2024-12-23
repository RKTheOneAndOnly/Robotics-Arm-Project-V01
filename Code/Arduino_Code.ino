/*************************************************************************************************************
⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
⭐Ensure to set the Parking positions so as to ensure no colission 💥💥💥                                 ⭐
⭐Set Max and Min limit for each of the motors                                                              ⭐      
⭐Motor 3 is reverse of Motor 2 i.e., it should rotate reverse of Motor 2 ⚠️⚠️⚠️                           ⭐
⭐                                                                                                          ⭐
⭐                                                                                                         ⭐
⭐                                                                                                         ⭐
⭐~21-12-2024 10:51 AM                                                                                     ⭐
⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
**************************************************************************************************************/

#include "HCPCA9685.h"
#include "A4988.h"

#define  I2CAdd 0x40
HCPCA9685 HCPCA9685(I2CAdd);

int MOTOR_POSITIONS[5]; // Array that returns the motor positions

// ----------------------------------------------------------------------
// Define the Parameters
// ----------------------------------------------------------------------
// SERVO 1 MG995
const int SERVO_1_CHANNEL = 0; // PCA9685 channel for the servo 1
const int SERVO_1_PARKING_ANGLE = 60; // Default parking position
const int SERVO_1_MIN_LIMIT = 30; // Min angle limit
const int SERVO_1_MAX_LIMIT = 150; // Max angle limit
const int SERVO_1_MIN = 0; // PWM Min value
const int SERVO_1_MAX = 400; // PWM Max Value
const int SERVO_1_SENSITIVITY = 10; // Step size for position adjustments for Servo 1

// SERVO 2 SG 90 Right Arm
const int SERVO_2_CHANNEL = 1; // PCA9685 channel for the servo 2
const int SERVO_2_PARKING_ANGLE = 45; // Default parking position
const int SERVO_2_MIN_LIMIT = 45; // Min angle limit
const int SERVO_2_MAX_LIMIT = 135; // Max angle limit
const int SERVO_2_MIN = 0; // PWM Min value
const int SERVO_2_MAX = 400; // PWM Max Value
const int SERVO_2_SENSITIVITY = 5; // Step size for position adjustments for Servo 2

// SERVO 3 SG 90 Left Asrm
const int SERVO_3_CHANNEL = 2; // PCA9685 channel for the servo 3
const int SERVO_3_PARKING_ANGLE = 180-SERVO_2_PARKING_ANGLE; // Default parking position
const int SERVO_3_MIN_LIMIT = SERVO_2_MIN_LIMIT ; // Min angle limit
const int SERVO_3_MAX_LIMIT = SERVO_2_MAX_LIMIT; // Max angle limit
const int SERVO_3_MIN = SERVO_2_MIN; // PWM Min value
const int SERVO_3_MAX = SERVO_2_MAX; // PWM Max Value
const int SERVO_3_SENSITIVITY = SERVO_2_SENSITIVITY; // Step size for position adjustments for Servo 3

// SERVO 4 SG 90 Gripper
const int SERVO_4_CHANNEL = 3; // PCA9685 channel for the servo 4
const int SERVO_4_PARKING_ANGLE = 0; // Default parking position
const int SERVO_4_MIN_LIMIT = 0; // Min angle limit
const int SERVO_4_MAX_LIMIT = 45; // Max angle limit
const int SERVO_4_MIN = SERVO_2_MIN; // PWM Min value
const int SERVO_4_MAX = SERVO_2_MAX; // PWM Max Value
const int SERVO_4_SENSITIVITY = 1; // Step size for position adjustments for Servo 4
bool SERVO_4_STATE = false; // Toggle Case to open and close gripper
int GRIP_STATE = 0 ; // To Get as array input

// STEPPER
#define MOTOR_STEPS 200
#define MAX_RPM 200
#define MOTOR_ACCEL 100 // Steps /s^2
#define MOTOR_DECEL 100 // Steps /s^2
#define MICROSTEPS 16 // Full Micro stepping Ensure in A4988 Board
#define DIR 2
#define STEP 3
#define MS1 10 // Is Hardwired
#define MS2 11 // Is Hardwired
#define MS3 12 // Is Hardwired
const int STEPPER_PARKING_POSITION = 0;
const int STEPPER_SENSITIVITY = 80; //Multiples of Microstepping pleae
int STEPPER_POSITION = 0;
A4988 stepper(MOTOR_STEPS, DIR, STEP, 1, MS1, MS2, MS3);



const int RESPONSE_TIME = 5; // Delay in milliseconds
const int SERVO_1_PARKING_POSITION = map(SERVO_1_PARKING_ANGLE, 0, 180, SERVO_1_MIN,SERVO_1_MAX);
const int SERVO_1_MIN_POSITION = map(SERVO_1_MIN_LIMIT, 0, 180, SERVO_1_MIN,SERVO_1_MAX);
const int SERVO_1_MAX_POSITION = map(SERVO_1_MAX_LIMIT, 0, 180, SERVO_1_MIN,SERVO_1_MAX);
const int SERVO_2_PARKING_POSITION = map(SERVO_2_PARKING_ANGLE, 0, 180, SERVO_2_MIN,SERVO_2_MAX);
const int SERVO_2_MIN_POSITION = map(SERVO_2_MIN_LIMIT, 0, 180, SERVO_2_MIN,SERVO_2_MAX);
const int SERVO_2_MAX_POSITION = map(SERVO_2_MAX_LIMIT, 0, 180, SERVO_2_MIN,SERVO_2_MAX);
const int SERVO_3_PARKING_POSITION = map(SERVO_3_PARKING_ANGLE, 0, 180, SERVO_3_MIN,SERVO_3_MAX);
const int SERVO_3_MIN_POSITION = map(SERVO_3_MIN_LIMIT, 0, 180, SERVO_3_MIN,SERVO_3_MAX);
const int SERVO_3_MAX_POSITION = map(SERVO_3_MAX_LIMIT, 0, 180, SERVO_3_MIN,SERVO_3_MAX);
const int SERVO_4_PARKING_POSITION = map(SERVO_4_PARKING_ANGLE, 0, 180, SERVO_4_MIN,SERVO_4_MAX);
const int SERVO_4_MIN_POSITION = map(SERVO_4_MIN_LIMIT, 0, 180, SERVO_4_MIN,SERVO_4_MAX);
const int SERVO_4_MAX_POSITION = map(SERVO_4_MAX_LIMIT, 0, 180, SERVO_4_MIN,SERVO_4_MAX);

int SERVO_1_POSITION = SERVO_1_PARKING_POSITION;
int SERVO_2_POSITION = SERVO_2_PARKING_POSITION;
int SERVO_3_POSITION = SERVO_3_PARKING_POSITION;
int SERVO_4_POSITION = SERVO_4_PARKING_POSITION;

void setup() {
    Serial.begin(9600);
    HCPCA9685.Init(SERVO_MODE);
    HCPCA9685.Sleep(false);

    // Set the servo to the parking position
   HCPCA9685.Servo(SERVO_1_CHANNEL,SERVO_1_POSITION); // ROT 1
   HCPCA9685.Servo(SERVO_2_CHANNEL,SERVO_2_POSITION); // ROT 2a
   HCPCA9685.Servo(SERVO_3_CHANNEL,SERVO_3_POSITION); // ROT 2b
   HCPCA9685.Servo(SERVO_4_CHANNEL,SERVO_4_POSITION); // GRIP

   //Stepper Initialisation
   stepper.begin(MAX_RPM, MICROSTEPS);
   stepper.enable(); // BASE
   stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        command = tolower(command); // Handle uppercase and lowercase commands
        // Switch Case command is used here
        switch (command) {
            case 'w': SERVO_1_MOVE(SERVO_1_POSITION + SERVO_1_SENSITIVITY); break;
            case 's': SERVO_1_MOVE(SERVO_1_POSITION - SERVO_1_SENSITIVITY); break;
            case 'a': 
                SERVO_2_MOVE(SERVO_2_POSITION + SERVO_2_SENSITIVITY); 
                SERVO_3_MOVE(SERVO_3_POSITION - SERVO_3_SENSITIVITY); 
                break;
            case 'd': 
                SERVO_2_MOVE(SERVO_2_POSITION - SERVO_2_SENSITIVITY); 
                SERVO_3_MOVE(SERVO_3_POSITION + SERVO_3_SENSITIVITY); 
                break;
            case '+': STEPPER_FORWARD(STEPPER_SENSITIVITY); break;
            case '-': STEPPER_BACKWARD(STEPPER_SENSITIVITY); break;
            case '0': 
                SERVO_1_MOVE(SERVO_1_PARKING_POSITION);
                SERVO_2_MOVE(SERVO_2_PARKING_POSITION);
                SERVO_3_MOVE(SERVO_3_PARKING_POSITION);
                SERVO_4_MOVE(SERVO_4_MIN_POSITION);
                STEPPER_PARK(); 
                break;
            default:
                Serial.println("Invalid command! Use 'W', 'S', 'A', 'D', '+', '-', or '0'.");  
        }

        if (command == 'o') {
            if (SERVO_4_STATE) {
                SERVO_4_MOVE(SERVO_4_MIN_POSITION);
                GRIP_STATE = 0;
            } else {
                SERVO_4_MOVE(SERVO_4_MAX_POSITION);
                GRIP_STATE = 1;
            }
            SERVO_4_STATE = !SERVO_4_STATE; // Toggle state
        }

        // Print motor angles
        int SERVO_1_ANGLE = map(SERVO_1_POSITION, SERVO_1_MIN, SERVO_1_MAX, 0, 180);
        int SERVO_2_ANGLE = map(SERVO_2_POSITION, SERVO_2_MIN, SERVO_2_MAX, 0, 180);
        int SERVO_3_ANGLE = map(SERVO_3_POSITION, SERVO_3_MIN, SERVO_3_MAX, 0, 180);
        int STEPPER_ANGLE = map(STEPPER_POSITION, 0, MOTOR_STEPS * MICROSTEPS, 0, 180);

        MOTOR_POSITIONS[0] = SERVO_1_ANGLE;
        MOTOR_POSITIONS[1] = SERVO_2_ANGLE;
        MOTOR_POSITIONS[2] = SERVO_3_ANGLE;
        MOTOR_POSITIONS[3] = STEPPER_ANGLE;
        MOTOR_POSITIONS[4] = GRIP_STATE;

        
        // Send the motor positions as an array
        Serial.print("Motor positions: [");
        Serial.print(MOTOR_POSITIONS[0]);
        Serial.print(", ");
        Serial.print(MOTOR_POSITIONS[1]);
        Serial.print(", ");
        Serial.print(MOTOR_POSITIONS[2]);
        Serial.print(", ");
        Serial.print(MOTOR_POSITIONS[3]);
        Serial.print(", ");
        Serial.print(MOTOR_POSITIONS[4]);
        Serial.println("]");
    }
}

// Move the servo to a specified position with bounds checking
void SERVO_1_MOVE(int POS1) {
    // Ensure the position is within the allowed range
    POS1 = constrain(POS1, SERVO_1_MIN_POSITION, SERVO_1_MAX_POSITION);
    // Update the servo position
    HCPCA9685.Servo(SERVO_1_CHANNEL, POS1);
    SERVO_1_POSITION = POS1;
    delay(RESPONSE_TIME); // Allow time for the servo to move
}
void SERVO_2_MOVE(int POS2) {
    // Ensure the position is within the allowed range
    POS2 = constrain(POS2, SERVO_2_MIN_POSITION, SERVO_2_MAX_POSITION);
    // Update the servo position
    HCPCA9685.Servo(SERVO_2_CHANNEL, POS2);
    SERVO_2_POSITION = POS2;
    // delay(0.01); // Allow time for the servo to move
}
void SERVO_3_MOVE(int POS3) {
    // Ensure the position is within the allowed range
    POS3 = constrain(POS3, SERVO_3_MIN_POSITION, SERVO_3_MAX_POSITION);
    // Update the servo position
    HCPCA9685.Servo(SERVO_3_CHANNEL, POS3);
    SERVO_3_POSITION = POS3;
    // delay(0.01); // Allow time for the servo to move
}
void SERVO_4_MOVE(int POS4) {
    // Ensure the target position is within the allowed range
    POS4 = constrain(POS4, SERVO_4_MIN_POSITION, SERVO_4_MAX_POSITION);

    // Gradually move the servo to the target position
    if (SERVO_4_POSITION < POS4) {
        for (int pos = SERVO_4_POSITION; pos <= POS4; pos += SERVO_4_SENSITIVITY) {
            HCPCA9685.Servo(SERVO_4_CHANNEL, pos);
            delay(1); // Allow time for the servo to move
        }
    } else if (SERVO_4_POSITION > POS4) {
        for (int pos = SERVO_4_POSITION; pos >= POS4; pos -= SERVO_4_SENSITIVITY) {
            HCPCA9685.Servo(SERVO_4_CHANNEL, pos);
            delay(0.1); // Allow time for the servo to move
        }
    }

    // Update the current position
    SERVO_4_POSITION = POS4;
}



//--------------------------------------------------------------------
// Stepper
//--------------------------------------------------------------------

void STEPPER_FORWARD(int steps) {
    STEPPER_POSITION += steps;
    stepper.move(steps);
    delay(RESPONSE_TIME);
}
void STEPPER_BACKWARD(int steps) {
    STEPPER_POSITION -= steps;
    stepper.move(-steps);
    delay(RESPONSE_TIME);
}
void STEPPER_PARK() {
    int steps_to_park = STEPPER_PARKING_POSITION - STEPPER_POSITION;
    STEPPER_POSITION = STEPPER_PARKING_POSITION;
    stepper.move(steps_to_park);
    delay(RESPONSE_TIME);
}