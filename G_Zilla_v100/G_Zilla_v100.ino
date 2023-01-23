/*

  G-Zilla Motion Controller: when it looks like it has already been done,
  but something was always off.

  I wrote this because I got tired of trying to adapt other people's solutions to my simple (?) needs.
  Maybe this is due to me not willing to adapt to other people's code, or ignorance, or nobody having the same goal as me.

  What this code is for:

    Receiving G code commands - G code like you see in 3D printer firmwares. List of codes below.
    Applying all safety measures I could think of to prevent out-of-bounds travel or unexpected conditions.
    Driving any (reasonable) number of motion axes using Step and Direction signals, with no position feedback
    Performing coordinated moves with all motors
    Keeping an eye out for motor error input, emergency stop input, and (fun feature) a "Freeze motion" input

  Why I wrote it;
  Despite my best effort, I have not found a motion controller code that is:
    Written for Arduino (accessible for me)
    Accepts G code as we know it from 3D printer firmwares as input
    Hardware agnostic (does NOT use low-level hardware-specific trickery to do the impossible)
    Runs on TEENSY 4.1 (which has enough IO and 600 MHz clock to sustain enough axes of motion)
    Supports more than 6 axes of motion easily (6 axis robot arm + track)
    Adaptable to slap on an inverse kinematics layer to control a robot arm (yes, that is in the works)

  If you know a controller that can do all this - DM me right now and I will happily use that.

  Limitations:
    Limited to only the commands necessary and required for robot arm movement (easy to add new commands)
    Limited to a specific G code format: G0 X25 Y-9 Z9 U300 V2 W-200 F4000 (omit what isn't needed)
    Serial feedback should be standardised and improved to be accepted by existing G code senders
    Inverse kinematics to be implemented: I have the math aready (big progress)

   Next level:
     Add inverse kinematics into this code, make IK calculationsbefore motor moves and forward kinematics for getting position
     Switch between direct joint control and IK control by G20 (direct joint pos) and G21 (IK)

     MIT license
     Written from the ground up by Denis Tikhonov in 5 days in 2023
*/

/*
   Does NOT support G code comments after ; or any other comment style
   Supported G and M codes:

   G0 - Coordinated move to specified position
   G1 - Coordinated move to specified position
   G4 - Dwell (wait)
   G27 - Go to predefined parking position
   G28 - Home all or selected axes
   G90 - Absolute positioning
   G91 - Relative positioning
   G92 - set current position

  M17 - enable motors
  M18 - disable motors
  M80 - enable motors
  M81 - disable motors
  M84 - disable motors
  M114 - report current position
  M119 - report endstop status

*/
#include <AccelStepper.h>

#define DEBUG // enables more serial output
#define UNSAFE_TESTING // disables E-stop and motor error checks for testing

// Number of stepper motors
// This code is intended for six, feel free to modify. Don't forget to edit the number of pins, G code letters, travel limits etc., and add an accelStepper instance.
// If you mess up this number OR gc_numLetters below, ALL HELL WILL BREAK LOOSE!!! You WILL start getting random errors which you never knew were possible, like a bool returning '62'
const int numMotors = 6;

// Pin definitions

const int TEST_LED_PIN = 13; // Active HIGH. For testing: shows the motors are moving
const int ENABLE_PIN = 24; // Active HIGH. Enable pin that turns on motor drives / turns off brakes. One pin for all axes, since you may lose known position when powering off a stepper motor.
const int ESTOP_PIN = 21; // Emergency stop input. Active HIGH. Should work at any point in the program IMMEDIATELY stopping all motion
const int FREEZE_PIN = 22; // Freeze movement input. Active HIGH. Different from Estop in that it just freezes movement without throwing errors or cancelling knowPosition, and only if/when a move is happening. It does not turn off motors or apply brakes.
const int MOTOR_ERROR_PIN = 23; // Pin for motor error input. Active HIGH. Unlike Estop, it gets ignored in the first couple seconds after motors are enabled, due to the behavior of the drives I am using. Error on any of the motors WILL cause a loss of known position for the entire robot arm, so behavior will be like Estop otherwise.

const int stepPins[numMotors] =         {2, 3, 4,  5,  6,  7};
const int dirPins[numMotors]  =         {8, 9, 10, 11, 12, 20};
const int limitSwitchPins[numMotors] =  {14, 15, 16, 17, 18, 19};

// Machine parameters (
const int homingDirection[numMotors] =    { -1, -1, -1, -1, -1, -1}; // MUST be 1 or -1
const int homingOrder[numMotors] =        {1, 3, 5, 4, 2, 0}; // Homing order: beginning to end. 0 is X axis, 5 is W axis. Must be only numMotors variables here!
const long homedPosition[numMotors]   = {0, 0, 0, 0, 0, 0};// Position of the arm when homing finished, after back-off. Must be within travel limits.

// PARK position is NOT homedPosition! This is a predefined position you can command to go to with G27.
const long parkPosition[numMotors]   = {50, 50, 50, 50, 50, 50};

const int backoffSteps = 10; // steps to back off a limit switch after homing
const int switchDebounce = 1; // milliseconds debounce time - after first trigger switch must be stable (still triggered) after this time in order to be registered
const int MOTOR_ERR_CLEAR_MS = 2000; // Used only in M17 Motors enable. Waits this time and checks if motor error input has bee cleared.
const int  SERIAL_BUF_LENGTH = 128; // Length of serial input buffer. Do not send longer lines of commands.

// Travel limits (soft limits) in steps
const long maxTravelLimits[numMotors] = {  1000,  1000,  1000,  1000,  1000,  1000};
const long minTravelLimits[numMotors] = { -1000, -1000, -1000, -1000, -1000, -1000};

// Number of G code letters: number of motors / axes + F (feedrate) value
// Added for code flexibility for different number of axes. Hopefully this will work fine
// int gc_numLetters = sizeof(gcLetters) / sizeof(gcLetters[0]);
const int gc_numLetters = numMotors + 1;
// Letters for axis naming in G code - used to compare
const char gcLetters[gc_numLetters] = {'X', 'Y', 'Z', 'U', 'V', 'W', 'F'};


// Max speed for all axes, steps/sec
const long maxSpeed = 1000;
// Homing speed for all axes, steps/sec
const int slowHomingSpeed = 50;

// Acceleration limit for all axes, steps/sec^2
const int maxAacceleration = 1000;
const int slowHomingAacceleration = 100;

// Feedrate set by G0/G1 command, steps/sec. You can set the default here, or send e.g. G0 F20 command after homing.
// By default, this or the last commanded feedrate, will be used for all moves
long feedrate = 100;

// G90/G91
bool positioningMode = 0; // 0 for absolute positioning, 1 for incremental positioning
/*
  Minimum pulse width - increase if step pulses are too fast for your motor driver. TB6600 are notorious for missing short pulses.
  Maximim theoretical pulse frequency (not factoring in processor performance) should be around 1000000/(2*pulseWidth), or around 250 kHz for 2 us pulses.
  However, your limiting factor will be processing power and number of motors: you should expect a maximum of 20-30 kHz from 8-bit Arduino
  boards such as Uno and Mega running 3 motors on a good day, with efficient code utilizing low-level hardware tricks.

  We are using accelStepper library for convenience, and it is reported to reliably give out 3200 - 4000 pulses per second to 3 motors on an Arduino Mega at 16 MHz.
  We are brute forcing it running on a Teensy 4.1 with 600 MHz clock, and with 6 motors. Dunno what to expect, maybe 100kHz best case scenario.

  If you are running into pulse frequency limitations, showing as glitching, sluttering, uneven moves or missed pulses, you can:
  (a) lower max speed, (b) lower the resolution (microstepping) of your drive, or (c) set the step input multiplier of your drive
  higher (e.g. input x 10). If you do any of this, do not forget to edit steps per unit above accordingly.
*/
const int minPulseWidth = 2; // microseconds

// Initialize stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper stepper2(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper stepper3(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper stepper4(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
AccelStepper stepper5(AccelStepper::DRIVER, stepPins[4], dirPins[4]);
AccelStepper stepper6(AccelStepper::DRIVER, stepPins[5], dirPins[5]);

// Array of stepper objects
// Obviously, needs to match numMotors[] length
AccelStepper steppers[] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6};

// Target positions for each axis in steps - do we need to initialize them?
long targetPositions[numMotors];

// Current positions for each axis in steps - do we need to initialize them?
long currentPositions[numMotors];

// Homing variables
int homingState = 0;
// This variable is used across the code to check if the robot has an established position and movement is alllowed.
/*
   Add knowPosition tracking across code. Maybe even tie it to a digital output for indication.
  Do not allow any movement other than homing (or jogging?) until position is established some way.
  If position is lost, stop all movement and report error immediately.

  Sources of known position:
  G28 homing
  G92 set absolute position

  Reasons to cancel knowPosition:
  Power ON
  Error on any motor driver
  No power on motor driver
  Disable motors
  Estop
  Errors in position calculation (current Position out of bounds)
  Hitting a limit switch - not included, as there are mid-travel limit switches in some cases

*/
bool knowPosition = 0;
bool eStop = 0;

#define ERR_OUT_OF_BOUNDS "Out of bounds!"
#define ERR_SPEED "Speed corrected"
#define ERR_INVALID_INPUT "Invalid input!"
#define ERR_POS_UNKNOWN "Not homed!"
#define ERR_HOMING_FAIL "Homing failed!"
#define ERR_POS_MISMATCH "Target not reached!"
#define ERR_MOTOR "Motor error!"
#define ERR_ESTOP "ESTOP!"

#define INFO_HOME_POS "Home Position "
#define INFO_PARK_POS "Park Position"
#define INFO_EST_POS "Established Position"

#define INFO_MOVE_START "Moving..."
#define INFO_FREEZE "Freeze motion..."
#define INFO_MOVE_FINISH "Arrived."
#define INFO_MOTOR_OFF "Motor OFF!"
#define ACK_OK "ok"
#define ACK_FAIL "fail"


void setup() {
  // Start listening for serial input
  Serial.begin(115200);
  delay(10);
  // If numMotors has been set to an insane value
  if (numMotors < 1 || numMotors > 32) {
    Serial.print("numMotors invalid!");
    while (1);
  }

  // Sizeof check - is a correct number of settings declared above?
  // Helps catch improper number of values in an array, most likely due to a typo
  if ( sizeof(stepPins) / sizeof(stepPins[0]) != numMotors ||
       sizeof(dirPins) / sizeof(dirPins[0]) != numMotors ||
       sizeof(limitSwitchPins) / sizeof(limitSwitchPins[0]) != numMotors ||
       sizeof(homingDirection) / sizeof(homingDirection[0]) != numMotors ||
       sizeof(homingOrder) / sizeof(homingOrder[0]) != numMotors ||
       sizeof(homedPosition) / sizeof(homedPosition[0]) != numMotors ||
       sizeof(parkPosition) / sizeof(parkPosition[0]) != numMotors ||
       sizeof(maxTravelLimits) / sizeof(maxTravelLimits[0]) != numMotors ||
       sizeof(minTravelLimits) / sizeof(minTravelLimits[0]) != numMotors ||
       sizeof(gcLetters) / sizeof(gcLetters[0]) != gc_numLetters)
  {
    Serial.print("sizeof check failed, check program!");
    while (1); // Hang forever, no point to continue
  }
  /*
    Sanity checks for all axes:  Are those inputs within expected range?
  */
  for (int i = 0; i < numMotors; i++) {

    if (minTravelLimits[i] >= maxTravelLimits[i]) {
      Serial.println("Travel limits incorrect!");
      while (1); // Hang forever, no point to continue
    }

    if (homedPosition[i] > maxTravelLimits[i] || homedPosition[i] < minTravelLimits[i]) {
      Serial.print(INFO_HOME_POS);
      Serial.println(ERR_OUT_OF_BOUNDS);
      while (1); // Hang forever, no point to continue
    }

    if (parkPosition[i] > maxTravelLimits[i] || parkPosition[i] < minTravelLimits[i]) {
      Serial.print("Parking position ");
      Serial.println(ERR_OUT_OF_BOUNDS);
      while (1); // Hang forever, no point to continue
    }

    if (homingOrder[i] > (numMotors + 1) || homingOrder[i] < 0) {
      Serial.print("Homing order ");
      Serial.println(ERR_INVALID_INPUT);
      while (1); // Hang forever, no point to continue
    }

    if (homingDirection[i] != -1 && homingDirection[i] != 1) {
      Serial.print("Homing dir ");
      Serial.println(ERR_INVALID_INPUT);
      while (1); // Hang forever, no point to continue
    }

    if (stepPins[i] > 64 || stepPins[i] < 0) { // Quite a stone-age method for checking pin validity... But hey, show me another one that works!
      Serial.print("step pin ");
      Serial.println(ERR_INVALID_INPUT);
      while (1); // Hang forever, no point to continue
    }
    if (dirPins[i] > 64 || dirPins[i] < 0) {
      Serial.print("dir pin ");
      Serial.println(ERR_INVALID_INPUT);
      while (1); // Hang forever, no point to continue
    }

    if (limitSwitchPins[i] > 64 || limitSwitchPins[i] < 0) {
      Serial.print("limit pin ");
      Serial.println(ERR_INVALID_INPUT);
      while (1); // Hang forever, no point to continue
    }

    if (limitSwitchPins[i] > 64 || limitSwitchPins[i] < 0) {
      Serial.print("limit pin ");
      Serial.println(ERR_INVALID_INPUT);
      while (1); // Hang forever, no point to continue
    }

  }// For 6 axes


  // Ohh man, after so many checks it feels like the stars should align in a specific way for this to ever reach program execution...
  // But no, it just requires correctly setting all values.

  // Now setting useful stuff
  for (int i = 0; i < numMotors; i++) {


    pinMode(limitSwitchPins[i], INPUT_PULLUP);  // Set all limit switch pins as inputs

    // Set max speed and acceleration for all steppers
    steppers[i].setMaxSpeed(maxSpeed);
    steppers[i].setAcceleration(maxAacceleration);
    steppers[i].setMinPulseWidth(2);

    targetPositions[i] = homedPosition[i];
    currentPositions[i] = homedPosition[i];
  }

  // Initialize inputs / outputs to correct SAFE states
  pinMode(TEST_LED_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(FREEZE_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ERROR_PIN, INPUT_PULLUP);

  digitalWrite(TEST_LED_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);

  // initialize with no known position so that the robot does not think it is homed.
  // It will only be set to 1 in either G92 function or after successful homing (G28)
  knowPosition = 0;
  positioningMode = 0;
  eStop = 0;

#ifdef UNSAFE_TESTING
  Serial.println("UNSAFE TESTING MODE");
  Serial.println("SAFETY DISABLED");
#endif
} // void setup

void loop() {
  processEstop();

  // Check if there is serial input
  if (Serial.available() > 0) {

    // Ignore all commands if E-stop is not cleared
    if (eStop) {
      Serial.println(ERR_ESTOP);
    } else {
      char cmdBuffer[SERIAL_BUF_LENGTH];
      int len = Serial.readBytesUntil('\n', cmdBuffer, SERIAL_BUF_LENGTH);
      cmdBuffer[len] = 0; // Null-terminate the string

      // For now, just treat G0 and G1 the same
      if (strstr(cmdBuffer, "G0") == cmdBuffer || strstr(cmdBuffer, "G1") == cmdBuffer) {

        // if there is no motor error and motors are ON, and if we know true position, accept move command. Otherwise don't waste time.
        if (motorError()) {
          Serial.println(ERR_MOTOR);
        } else if (motorEnabled()) {
          Serial.println(INFO_MOTOR_OFF);
          // if the robot does not know its position, prohibit all movement and throw an error.
        } else if (!knowPosition) {
          Serial.println(ERR_POS_UNKNOWN);
          // Maybe tell the user to home first. Do not perform any movement.
        } else { // G0 - No error, proceed with command processing

          bool isCmd[gc_numLetters]; // 1 if axis has had any update in the sent gcode
          long gc[gc_numLetters]; // axis values straight outta gcode number of motors plus feedrate
          // Initialize both local variables with zeros. Done this way to preserve flexibiity in number of Motors.
          for (int i = 0; i < gc_numLetters; i++) {
            isCmd[i] = 0;
            gc[i] = 0;
          }

          char *token = strtok(cmdBuffer, " ");

          while (token != NULL) { // strtok bites pieces (tokens) off the beginning of the buffer, keep looping until it has reached the end
            for (int i = 0; i < gc_numLetters; i++) { // 6 axis + feedrate. Their letters are determined by gcLetters,
              if (token[0] == gcLetters[i]) {// the received value is written into gc[], and isCmd[] flag is raised for the affected axes
                isCmd[i] = 1;
                gc[i] = atol(token + 1);  // This is the dangerous function - turns char into long
                break; // Absolutely required
              } // if token
            } // for int
            token = strtok(NULL, " "); // Reached buffer terminator
          } // while token

          // Write received values to targetPositions, making a sanity check and taking into account G90/G91
          for (int i = 0; i < numMotors; i++) {
            // Sanity check for gcode raw values; do not write invalid values to working variables
            if (isnan(gc[i]) || isinf(gc[i])) {
              Serial.println(ERR_INVALID_INPUT);
              Serial.println("in axis");
            } else { // if we received an insane value, else...
              if (isCmd[i]) { // only if the specific axis command has been included in the code
                if (positioningMode == 0) { // Absolute positioning (G90)
                  targetPositions[i] = gc[i];
                } else { // Incremental positioning (G91)
                  targetPositions[i] = currentPositions[i] + gc[i]; // SOMETHING FISHY HAPPENS HERE
                }
              } // if isCmd
            } // else - we received a sane value
          } // for loop - repeat numMotors times

          // There is a sanity check above but there is no check for position within travel limits.
          // This is because a check for that is applied to targetPositions later in enforceLimits()
          // 10 lines of code below, the target positions are forced within travel boundaries and an Out of bounds message is given.

          // Set feedrate if it was provided and if it has a sane value
          // This function kinda hopes that there is only one additional G code letter apart from the motors, and it is the feedrate.
          // If you add some other functionality, change it here too.
          if (isCmd[numMotors]) { // if we received a feedrate command, update. If not, keep as is.
            if (isnan(gc[numMotors]) || isinf(gc[numMotors]) || gc[numMotors] < 0.1) {
              Serial.println(ERR_INVALID_INPUT); // Throws error if there is no feedrate commabded
              Serial.println("in feedrate");
            } else { // if is invalid, throw an error. If valid, set feedrate.
              feedrate = gc[numMotors];
            }
          } // else
          // Same as for positions, the feedrate is later checked by enforceLimits() and forced to be above 1 and below maxSpeed.
          // However, no message is given due to low (seeming) significance.

          // Super important: perform a check if commanded values are within limits. Force them to be within the limits. Issue relevant warnings.
          // Important: this function overwrites targetPositions[] and is placed here, before comparing current and target positions after the move.
          // In theory, it should not affect that comparison, and isPositionAsCommanded() should not be affected.

          enforceLimits();

          // Move motors here
          moveMotorsToTarget();
          // That's it, moveMotors takes targetPositions[] and runs the motors to them from currentPositions

        } // if there is no motor error and motors are ON
      } // G0 / G1 move

      // G4 Dwell
      //      G4 P500 // Dwell for 1/2 second
      //      G4 S3 // Dwell for 3 seconds
      if (strstr(cmdBuffer, "G4") == cmdBuffer) {
        int ms = 0;
        int sec = 0;

        char *token = strtok(cmdBuffer, " ");
        while (token != NULL) { // written the old way
          if (token[0] == 'P') {
            ms = atol(token + 1);
            break;
          } else if (token[0] == 'S') {
            sec = atol(token + 1);
            break;
          }
          token = strtok(NULL, " ");
        }

        if (isnan(ms) || isinf(ms) || isnan(sec) || isinf(sec) )  {
          Serial.println(ERR_INVALID_INPUT);
        } else {// if value is valid
          if (ms < 0 || ms > 30000 || sec < 0 || sec > 60) { //
            Serial.println("Too long");
          } else {
#ifdef DEBUG
            Serial.print("Wait ");
            Serial.print(sec);
            Serial.print(" sec ");
            Serial.print(ms);
            Serial.println(" ms...");
#endif
            // Copying Marlin behavior: S has precedence over P
            if (sec > 0) delay(sec * 1000); // seconds to ms
            else delay(ms);
            Serial.println(ACK_OK);
          } // else if not too long or short
        } // else if value is valid
      }  // G4 Dwell


      // G27 - Go to parking position
      if (strstr(cmdBuffer, "G27") == cmdBuffer) {

        // if there is no motor error and motors are ON, and if we know true position, accept move command. Otherwise don't waste time.
        if (motorError()) {
          Serial.println(ERR_MOTOR);
        } else if (motorEnabled()) {
          Serial.println(INFO_MOTOR_OFF);
          // if the robot does not know its position, prohibit all movement and throw an error.
        } else if (!knowPosition) {
          Serial.println(ERR_POS_UNKNOWN);
          // Maybe tell the user to home first. Do not perform any movement.
        } else { // G27 - No error, proceed with command processing

          for (int i = 0; i < numMotors; i++) {
            targetPositions[i] = parkPosition[i];
          }

          // Move motors here - the same code as G0
          moveMotorsToTarget();

        } // else if no error
      } // G27


      // G28 homing
      if (strstr(cmdBuffer, "G28") == cmdBuffer) {
        knowPosition = 0; // If we have initiated homing, we immediately do not have a known position, bc even a plain G28 command will cause movement on some axes
        // if there is no motor error and motors are ON, and if we know true position, accept move command. Otherwise don't waste time.
        if (motorError()) {
          Serial.println(ERR_MOTOR);
        } else if (motorEnabled()) {
          Serial.println(INFO_MOTOR_OFF);
          // if the robot does not know its position, prohibit all movement and throw an error.
        } else { // G28 - No error, proceed with command processing

          bool err = 0;
          bool isCmd[numMotors]; // 1 if axis has had any update in the sent gcode
          int numCmd = 0; // counter of axis home commands issued
          int numHomed = 0; // Counter of axis actually homed
          // Initialize local variables with zeros. Done this way to preserve flexibiity in number of Motors.
          for (int i = 0; i < numMotors; i++) {
            isCmd[i] = 0;
          }
          char *token = strtok(cmdBuffer, " ");

          while (token != NULL) { // strtok bites pieces (tokens) off the beginning of the buffer, keep looping until it has reached the end
            for (int i = 0; i < numMotors; i++) { // 6 axis. Their letters are determined by gcLetters,
              if (token[0] == gcLetters[i]) {//  isCmd[] flag is raised for the affected axes - the result is an arrray of bools for those axes which need to be homed.
                isCmd[i] = 1;
                numCmd++; // Count axes that have been mentioned. Later will be used to determine if all commanded axes have been homed.
                break; // Absolutely required
              } // if token
            } // for int
            token = strtok(NULL, " "); // Reached buffer terminator
          } // while token

          if (numCmd == 0) { // If we received a plain G28 command (no letters mentioned), we need to home all axes
            for (int i = 0; i < numMotors; i++) { // 0 thru 5
              isCmd[i] = 1;
              numCmd++; // Count axes that have been mentioned. Later will be used to determine if all commanded axes have been homed.
            }
          }
#ifdef DEBUG
          Serial.print("Homing ");
          Serial.print(numCmd);
          Serial.println(" axes. Please wait...");
#endif
          // Home all axes in order, if there are no errors
          for (int i = 0; i < numMotors; i++) { // 0 thru 5
            processEstop(); // check estop condition before homing every axis, since it might have been triggered within axis movement
            if (!eStop && !err && !motorError()) {
              Serial.print(i + 1); // position in array into human readable format
              Serial.print(" - ");
              // homingOrder[i] gives axis numbers in the correct order for homing
              if (isCmd[homingOrder[i]]) {
                //Serial.println(gcLetters[homingOrder[i]]); // axis letter for the axis being homed
                if (homeAxis(homingOrder[i])) numHomed++; //Perform homing sequence and increase homed axes counter, return error if unsuccessful
                else err = true; // If commented out, allows attempts for all axes
              } // if iscmd
            } //
          } // home all axes in order.

          if (numHomed < 1) { // If no axes homed successfully
            knowPosition = 0;
            Serial.println(ERR_HOMING_FAIL);
          } else if (numCmd != numHomed) { // If not all commanded axes homed
            knowPosition = 0;
            Serial.println(ERR_HOMING_FAIL);
          } else if (numCmd == numHomed) { // if all commanded axes homed successfully
            knowPosition = 1;
            Serial.println(ACK_OK);
            Serial.println(INFO_EST_POS);
          }
          // additional info
#ifdef DEBUG
          if (numHomed < 1) { // If no axes homed successfully
            Serial.println("No axis homed");
          } else if (numCmd != numHomed) { // If not all commanded axes homed
            Serial.print(numHomed);
            Serial.println(" axes homed");
          } else if (numCmd == numHomed) { // if all commanded axes homed successfully
            Serial.println("Homed");
            Serial.print(numCmd);
            Serial.println(" axes. ");
          }
#endif
        } // if there is no motor error
      } // G28 homing

      if (strstr(cmdBuffer, "G90") == cmdBuffer) {
        positioningMode = 0; // Absolute positioning
        Serial.println(ACK_OK);
      }
      if (strstr(cmdBuffer, "G91") == cmdBuffer) {
        positioningMode = 1; // Incremental positioning
        Serial.println(ACK_OK);
      }

      // G92 - set current position. I also used it to be a source of knowPosition if all axes were correctly sent.
      // Will be useful for testing purposes when homing switches are not yet dealt with.
      if (strstr(cmdBuffer, "G92") == cmdBuffer) {
        bool err = 0; // Boolean to raise if there was an error
        int cmd = 0; // Boolean to sum all commanded axes. Will not establish positions if not all axes have been commanded.
        bool isCmd[numMotors]; // 1 if axis has had any update in the sent gcode
        long gc[numMotors]; // axis values straight outta gcode - only 6 values here as this has nothing to do with feedrate
        // Initialize both local variables with zeros. Done this way to preserve flexibiity in number of Motors.
        for (int i = 0; i < numMotors; i++) {
          isCmd[i] = 0;
          gc[i] = 0;
        }
        char *token = strtok(cmdBuffer, " ");

        while (token != NULL) { // strtok bites pieces (tokens) off the beginning of the buffer, keep looping until it has reached the end
          for (int i = 0; i < numMotors; i++) { // 6 axis + feedrate. Their letters are determined by gcLetters,
            if (token[0] == gcLetters[i]) {// the received value is written into gc[], and isCmd[] flag is raised for the affected axes
              isCmd[i] = 1;
              gc[i] = atol(token + 1);  // This is the dangerous function - turns char into long
              break; // Absolutely required
            } // if token
          } // for int
          token = strtok(NULL, " "); // Reached buffer terminator
        } // while token

        // Write received values to targetPositions, making a sanity check and taking into account G90/G91
        for (int i = 0; i < numMotors; i++) {
          // Sanity check for gcode raw values; do not write invalid values to working variables
          if (isnan(gc[i]) || isinf(gc[i])) {
            Serial.println(ERR_INVALID_INPUT);
            err = true; // raise error flag if any value was invalid. We cannot set
          } else { // if we received an insane value, else...
            if (isCmd[i]) { // only if the specific axis command has been included in the code
              if (gc[i] >= minTravelLimits[i] && gc[i] <= maxTravelLimits[i]) { // if commanded position is within travel limits
                cmd++; // Sum the number of axes which have received a command
                // Do nothing else here - first finish all error checks for all axes, then start again and write values
              } else {
                Serial.print(ERR_OUT_OF_BOUNDS);
                err = true;
              }
            } // if isCmd
          } // else - we received a sane value
        } // for loop - repeat 6 times

        if (!err) {
          for (int i = 0; i < numMotors; i++) {
            if (isCmd[i]) { // only if the specific axis command has been included in the code
              currentPositions[i] = gc[i]; // No check for feedrate here
              targetPositions[i] =  gc[i];
              steppers[i].setCurrentPosition(gc[i]); // Need to investigate if the output of this function of accelstepper is actually usable for this purpose.
            } // if is command
          } // for all axes
          // When data for all axes is in place with no errors, set knowPosition to 1. If not all axes have been commanded,
          // we still srite their values, but it is not a source of known position.
          if (cmd > 0) Serial.println(ACK_OK); // Only respond with OK if at least one axis received command
          if (cmd == numMotors) { // if all axes were homed
            knowPosition = 1;
            Serial.println(INFO_EST_POS);
          }
        } // if no error

      } // G92 set position

      // M17, M80 - enable motors
      if (strstr(cmdBuffer, "M17") == cmdBuffer || strstr(cmdBuffer, "M80") == cmdBuffer) {
        digitalWrite(ENABLE_PIN, HIGH);
        delay(MOTOR_ERR_CLEAR_MS);
        if (!motorError()) { // if motor driver error has cleared in time
          Serial.println(ACK_OK);
        } else {
          digitalWrite(ENABLE_PIN, HIGH);
          Serial.println(ERR_MOTOR);
        }
      } // M17 enable motors

      // M18, M81, M84 - disable motors
      if (strstr(cmdBuffer, "M18")  == cmdBuffer || strstr(cmdBuffer, "M81")  == cmdBuffer || strstr(cmdBuffer, "M84") == cmdBuffer) {
        knowPosition = 0;
        for (int i = 0; i < numMotors; i++) {
          steppers[i].stop();
        } // repeat for all motors
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println(ACK_OK);
      } // M18 / M84 disable motors

      // M114 - Report current position
      // X:0.00 Y:0.00 Z:0.00
      if (strstr(cmdBuffer, "M114") == cmdBuffer) {
        for (int i = 0; i < numMotors; i++) {
          delay(1);
          Serial.print(gcLetters[homingOrder[i]]); // axis letter
          Serial.print(":");
          Serial.print(currentPositions[i]);
          if (i < numMotors - 1) Serial.print(" ");
          else Serial.println(); // end with newline
        }
      } // M114 report pos

      // M119 - Report endstop status
      // X:H Y:L etc.
      if (strstr(cmdBuffer, "M119") == cmdBuffer) {
        for (int i = 0; i < numMotors; i++) {
          delay(1);
          Serial.print(gcLetters[homingOrder[i]]); // axis letter
          Serial.print(":");
          Serial.print(digitalRead(limitSwitchPins[i]));
          if (i < (numMotors - 1)) Serial.print(" ");
          else Serial.println(); // end with newline
        }
      } // M119 endstop status


    } // else - no estop
  } // if serial available

  delayMicroseconds(1); // idle
} // loop

/*
   Process E-Stop. Setst eStop bool to HIGH or LOW depending on whether input is present.
   Adds debounce time to minimize false triggers.
   Elegantly written to execute as fast as possible in time-critical loops.
*/
void processEstop() {
  if (digitalRead(ESTOP_PIN) == HIGH) { // For limit switches active HIGH and normally LOW
    delay(switchDebounce); // wait debounce time and check again.
    if (digitalRead(ESTOP_PIN) == HIGH) { // For limit switches active HIGH and normally LOW
      eStop = true;
    }
  } else if (eStop) { // if eStop is active, give it a chance to undo itself if the input has been cleared.
    if (digitalRead(ESTOP_PIN) == LOW) {
      delay(switchDebounce); // wait debounce time and check again.
      if (digitalRead(ESTOP_PIN) == LOW) {
        eStop = false;
      }
    }
  }
#ifdef UNSAFE_TESTING
  eStop = false; // UNSAFE - FOR TESTING
#endif

  // Apply immediate actions if estop occured: stop and disable all motors without delay, reset knowposition to false, throw error
  if (eStop) {
    for (int i = 0; i < numMotors; i++) {
      steppers[i].stop();
    }
    digitalWrite(ENABLE_PIN, LOW);
    knowPosition = 0;
    Serial.println(ERR_ESTOP);
  }
} // process estop

// Freeze command: use this in
void procssFreeze() {
  bool already = 0;
  while (digitalRead(FREEZE_PIN) == HIGH) {
    delay(1);
    if (!already) {
      already = true;
      Serial.println(INFO_FREEZE);
    }
  }
} // freeze

// Check if motor is enabled from Enable pin
bool motorEnabled() {
  return (digitalRead(ENABLE_PIN));
}

// Motor error check: reads motor error input pin and rechecks after debounce time to report reliably.
// Can be set to always return false for testing.
bool motorError() {
#ifdef UNSAFE_TESTING
  return false; // UNSAFE - FOR TESTING
#endif
  if (digitalRead(MOTOR_ERROR_PIN) == HIGH) { // For limit switches active HIGH and normally LOW
    delay(switchDebounce); // wait debounce time and check again.
    if (digitalRead(MOTOR_ERROR_PIN) == HIGH) { // For limit switches active HIGH and normally LOW
      Serial.println(ERR_MOTOR);
      return true;
    }
  } else return false; // if we have not verified that motor error pin stays high, return false
} // motorerror

/*
   Checks if target position set in G code is within travel limits. If it is not, forces the value within limits.
   Only gives warnings without causing big errors.
   WARNING: this check is not exhaustive and it is still possible to trick the code into going out of bounds
   if you set G92 to a wrong position (within limits) and then command a move greater than the physical
   distance to end of travel from commanded position.
*/
void enforceLimits() {
  for (int i = 0; i < numMotors; i++) {
    if (targetPositions[i] > maxTravelLimits[i]) {
      targetPositions[i] = maxTravelLimits[i];
      Serial.println(ERR_OUT_OF_BOUNDS);
    } else if (targetPositions[i] < minTravelLimits[i]) {
      targetPositions[i] = minTravelLimits[i];
      Serial.println(ERR_OUT_OF_BOUNDS);
    }
  }

  if (feedrate > maxSpeed) {
    Serial.println(ERR_SPEED);
    feedrate = maxSpeed;
  }
  else if (feedrate < 1) {
    Serial.println(ERR_SPEED);
    feedrate = 1;
  }
} // enforce limits

/*
  Move motors to target - pretty self-explanatory. this function plans the move, moves motors to the target position
  and verifies if the target has been reached. It relies on YOU to check if the motors are enabled and there is no error,
  BEFORE calling this function.
*/
void moveMotorsToTarget() {

  Serial.println(INFO_MOVE_START);
  // Move the motors to the commanded positions
  planMotors();

  while (!isMovementComplete()) { // this bool function also contains the motor run command, which must be run in a very fast loop
    processEstop();
    if (eStop) break;
    procssFreeze(); // Can freeze movement while FREEZE input is present. Does not affect any functionality or cause errors.
    digitalWrite(TEST_LED_PIN, HIGH);
    // 'Hang' the processor (and comms) until movement is finished. Maybe it's a bad idea, but... Who knows.
    // Prevents user from executing commands while running - be careful: if commands get sent before ACK for finished move,
    // they may pile up in the receive buffer and instantly execute once move is finished.
    // May implement waiting for the move to finish in a different way, in a flashy nonblocking manner.
  }
  digitalWrite(TEST_LED_PIN, LOW);

  // Two checks one inside the other. If I was confident in the correctness of all my code and reliability of string parsing, these checks would not be needed.
  // They are here for added safety if something goes wrong in the calculations.
  if (isPositionAsCommanded()) { // Checks if position has been reached.
    if (isCurrentPosinLimits()) { // Checks if current position is within travel lmits
      // Send acknowledgement of successfully finished movement
      Serial.println(INFO_MOVE_FINISH);
      // No other actions, processor is free to execute other stuff
    } else { // If  ANY axis' current position is outside of travel limits by any reason (most likely math errors or unintended writing to position variables)
      // Move error: axis gone out of bounds. Throw error and reset knowposition to 0.
      // This should prevent further movement, and require a G92 command or a successful homing procedure to proceed.
      knowPosition = 0;
      Serial.print(ERR_OUT_OF_BOUNDS);
      Serial.println(ERR_POS_UNKNOWN);
    }
    // No other actions, processor is free to execute other stuff
  } else { // If there is a difference in ANY axis' current position and commanded position after the move
    // Move error: position reached does not match commanded position. Throw error and reset knowposition to 0.
    // This should prevent further movement, and require a G92 command or a successful homing procedure to proceed.
    knowPosition = 0;
    Serial.println(ERR_POS_MISMATCH);
    Serial.println(ERR_POS_UNKNOWN);
  }
}

/*
   Plan motors function - takes the commanded feedrate, set acceleration and commanded target position
   which have already been sanitu checked and prepared by other functions, calculates trapezoidal
   motion profile (supposedly) and writes all this as accelStepper library inputs. Then starts all motors at the same time.
   This is a nonblocking function and should execute almost instantly. Motors are run by a different function, bool isMoveComplete
*/
void planMotors() {
  long distance, speed;
  long acceleration = maxAacceleration;
  // Set the speed and acceleration for each stepper, set target position
  for (int i = 0; i < numMotors; i++) {
    distance = targetPositions[i] - currentPositions[i];
    if (distance != 0) { // IMPORTANT: If distance is not zero, then proceed with calculations. If you set speed to zero, you will see no movement. Don't ask me how I know.
      speed = sqrt(2 * acceleration * abs(distance));

      /*
        This way, if the commanded feedrate is lower than the calculated speed, the stepper motors will move
        at the commanded feedrate and the acceleration  will be adjusted accordingly to ensure that the stepper
        motors reach the target position at the same time.
      */
      if (speed > feedrate) { // if calculated speed higher than commanded feedrate
        speed = feedrate;
        acceleration = speed * speed / (2 * abs(distance)); // recalculate acceleration to keep sync
      }
      steppers[i].setMaxSpeed(speed);
      steppers[i].setAcceleration(acceleration);
      //steppers[i].move(distance);
      steppers[i].moveTo(targetPositions[i]);
    } // If distance is not zero
  } // for loop  repeats for every motor

} // planMotors

//  Runs all motors at speeds and accelerations set above to commanded position. Needs to be called in a loop many times until returns true.
// Done in a separate loop to prevent delays in starting motors while calculations above are being performed.
// Called from G1/G0 move command where special motion control is not needed - just set targets and move until we reach them (hoping the steppers don't skip a pulse)
bool isMovementComplete() {
  int temp = 0;
  for (int i = 0; i < numMotors; i++) {
    steppers[i].run();
    //  if (steppers[i].distanceToGo() != 0) temp++; // Not used, as it may slightly differ from zero with float calculations
    if (steppers[i].isRunning() != 0) temp++;
  } // repeat for all motors

  if (temp == 0) // if no axis is moving
    return true;
  else
    return false;
} // ismovementcomplete

/*
   Using this function now to check if we reached target position
   Is position as commanded? You might think it should be once the motors have stopped moving after accelStepper has done its work and they have stopped at their destination.
   But who knows? Maybe there has been some computation glitch, or the motors still have not stopped (why?), or something else has happened.
   It would be logical to automatically throw a knowPosition error if this function returns false, but we'll leave this decision to other functions.
   By the way, this function may behave weird upon startup, homing or when a position is commanded via G92.
   We need to account for that and always set current position AND target position equal after those actions.
*/
bool isPositionAsCommanded() {
  int diff = 0; // counter for errors
  // Update the currentPositions[] array
  for (int i = 0; i < numMotors; i++) {
    currentPositions[i] = steppers[i].currentPosition(); // Need to investigate if the output of this function of accelstepper is actually usable for this purpose.
    if (abs(currentPositions[i] - targetPositions[i]) > 0) diff++; // Depending on results we might want to increase the permitted error. Though, we are working with no position feedback and only with calculated values, difference should not be high. if you are constantly getting a high difference you might want to seek errors somewhere.
  } // repeat for all axes
  if (diff == 0) // if current position matches target with a relatively high precision (set above)
    return true;
  else
    return false;
} // is position as commanded

/*
   Is current position within travel limits?
   It would be logical to assume that it would be, especially if we are doing a limits check on target position, but you know... things happen.
   If at any point in time current position is out of travel limits, throw knowPosition error, stop all movement
   May be false triggered by: initializing variables at startup to values out of limits, weird output from acccelStepper.currentposition function, or wrong calculations around steps per unit.
   or getting very close to travel limits
*/
bool isCurrentPosinLimits() {
  int out = 0; // counter for errors
  // Update the currentPositions[] array
  for (int i = 0; i < numMotors; i++) {
    currentPositions[i] = steppers[i].currentPosition(); // Need to investigate if the output of this function of accelstepper is actually usable for this purpose.
    if (currentPositions[i] > maxTravelLimits[i] || currentPositions[i] < minTravelLimits[i]) out++; // if less than minimum OR higher than maximum
  } // repeat for all axes
  if (out == 0) // if current position is within motion limits
    return true;
  else
    return false;
} // isCurrentPosinLimits

/*
   Homing procedure for a specified axis.
   What is this if not perfection?

   Logic of operation:

   (1) Set acceleration to max (default) and speed to a fraction of max speed
   (2) Command a move in the homing direction, with a maximum length of 1.5 times the full travel length of an axis.
   (3) First apprach: Start the move, keeping an eye out for E-stop triggers or homing switch triggers. If home switch is already triggered on the beginning of the move, skip to backoff.
   (4) As soon as homing switch is triggered, stop movement. If switch has not been triggered over the entire 1.5 x travel (which should have happened!), throw error.
   (5) Set slow homing speed and acceleration
   (6) First backoff: move in the direction opposite to homing direction for a specified number of steps. If homing switch is still triggered (when it should not be!), throw error.
   (7) Command a move in the homing direction, several times the backoff steps - in case the limit switch reads inconsistently by several steps
   (8) Second approach: Start the move, keeping an eye out for E-stop triggers or homing switch triggers.
   (9) As soon as homing switch is triggered, stop movement. If switch has not been triggered over the entire (several times the backoff) travel (which should have happened!), throw error.
   (10) Second backoff: move in the direction opposite to homing direction for a specified number of steps. If homing switch is still triggered (when it should not be!), throw error.
   (11) Return speed and acceleration settings to normal
   (12) In case of homing without errors, write homedPositions to current and target positions for this axis and return true, in case of estop or any error write nothing and return false
*/
bool homeAxis(int num) {
  bool error = 0;
  /*
     error - internal error flag. Triggered by :
     Estop (obviously)
     Not finding limit switch across 1.5 times travel length
     Homing switch still triggered after back-off
     Homing switch not found on 2nd approach
     Homing switch still triggered after 2nd backoff

     All these conditions, given correct settings, are abnormal, and directly or indirectly show there is a problem with motors or the limit switch
  */
  // Approach limit switch fast. If limit switch not found within 1.5 times max travel limit, throw error.
  steppers[num].setAcceleration(maxAacceleration);
  steppers[num].setMaxSpeed(maxSpeed * 0.3); // Limit switch fast seek speed

  /*
      First approach
     ---------------------------------------------------------------------
  */
  // set the length of this move to 1.5 full travel lengths, and take into account homing direction.
  steppers[num].move( homingDirection[num] * 1.5 * ( maxTravelLimits[num] - minTravelLimits[num] ));

  // CHECK: Does distanceToGo actually work this way? Do we need to add limit switch debounce to minimize false triggers?
  //  Move to that distance, and if an endstop has not been found, throw an error that something is wrong
  while (steppers[num].distanceToGo() != 0) {
    processEstop();
    if (eStop) {
      steppers[num].stop(); // CHECK: Does stop() also remove any distance left to go?
      error = 1;
      break; // Break the loop if switch was pressed. This way, error = 0 stays and enables other actions.
    } else { // no estop
      if (digitalRead(limitSwitchPins[num]) == HIGH) { // For limit switches active HIGH and normally LOW
        delay(switchDebounce); // wait debounce time and check again. If switch is still high, stop movement, clear error and continue sequence. If not, continue while loop.
        if (digitalRead(limitSwitchPins[num]) == HIGH) { // For limit switches active HIGH and normally LOW
          steppers[num].stop(); // CHECK: Does stop() also remove any distance left to go?
          error = 0; // Only returns no error if switch stays reliably triggered
          break; // Break the loop if switch was pressed. This way, error = 0 stays and enables other actions.
        } else error = 1; // We are not checking for error in while loop, so it will onlt have importance if while loop never gets interrupted and reaches 1.5 times travel limit without encountering an endstop.
      } else error = 1; // We are not checking for error in while loop, so it will onlt have importance if while loop never gets interrupted and reaches 1.5 times travel limit without encountering an endstop.
    } // else no estop
    digitalWrite(TEST_LED_PIN, HIGH);
    steppers[num].run(); // run motors in the end - loop would've been broken by now if it wasn't required
  } // while loop
  digitalWrite(TEST_LED_PIN, LOW);

  if (!eStop && !error) { // if no error after first approach
    /*
        First backoff
       ---------------------------------------------------------------------
    */
    delay(1); // 1ms delay - just in case

    // Super slow speed from here to confirm limit switch position
    steppers[num].setAcceleration(slowHomingAacceleration);
    steppers[num].setMaxSpeed(slowHomingSpeed);
    // Move a distance of backoffsteps OPPOSITE to homing direstion and check if the switch has been released
    steppers[num].move( -1 * homingDirection[num] * backoffSteps); //

    while (steppers[num].distanceToGo() != 0) {
      processEstop();
      if (eStop) { // check for estop, then run motor
        steppers[num].stop(); // CHECK: Does stop() also remove any distance left to go?
        error = 1;
        break; // Break the loop if switch was pressed. This way, error = 0 stays and enables other actions.
      } else { // no estop
        digitalWrite(TEST_LED_PIN, HIGH);
        steppers[num].run();
      }
    }
    digitalWrite(TEST_LED_PIN, LOW);

    steppers[num].stop();
    delay(1); // 1ms delay - just in case

    // Check the limit switch at the end of each homing move
    // If endstop is still triggered after backoff
    // Funnily enough, just setting this and the other post-backoff check to LOW emulates a successful homing sequence
    if (digitalRead(limitSwitchPins[num]) == HIGH) {
      error = 1; // switch still pressed after 1st back-off
    }

    if (!eStop && !error) { // If no error after first backoff
      /*
          Second approach
         ---------------------------------------------------------------------
      */
      // set the length of this move to 10x backoffsteps in the homing direction (toward limit switch), in case limit switch is not very precise
      steppers[num].move( homingDirection[num] * 10 * backoffSteps);

      // CHECK: Does distanceToGo actually work this way? Do we need to add limit switch debounce to minimize false triggers?
      while (steppers[num].distanceToGo() != 0) { //  Move to that distance, and if an endstop has not been found, throw an error that something is wrong
        processEstop();
        if (eStop) {
          steppers[num].stop(); // CHECK: Does stop() also remove any distance left to go?
          error = 1;
          break; // Break the loop if estop was pressed. This way, error = 0 stays and enables other actions.
        } else { // no estop
          if (digitalRead(limitSwitchPins[num]) == HIGH) { // For limit switches active HIGH and normally LOW
            delay(switchDebounce); // wait debounce time and check again. If switch is still high, stop movement, clear error and continue sequence. If not, continue while loop.
            if (digitalRead(limitSwitchPins[num]) == HIGH) { // For limit switches active HIGH and normally LOW
              steppers[num].stop(); // CHECK: Does stop() also remove any distance left to go?
              error = 0; // Only returns no error if switch stays reliably triggered
              break; // Break the loop if switch was pressed. This way, error = 0 stays and enables other actions.
            } else error = 1; // We are not checking for error in while loop, so it will onlt have importance if while loop never gets interrupted and reaches end of move  without encountering an endstop.
          } else error = 1; // We are not checking for error in while loop, so it will onlt have importance if while loop never gets interrupted and reaches end of move  without encountering an endstop.
        } // else no estop
        digitalWrite(TEST_LED_PIN, HIGH);
        steppers[num].run(); // run motors in the end - loop would've been broken by now if it wasn't required
      } // while loop
      digitalWrite(TEST_LED_PIN, LOW);

      /*
          Second backoff
         ---------------------------------------------------------------------
      */
      if (!eStop && !error) {
        delay(1); // 1ms delay - just in case
        // Super slow speed from here to confirm limit switch position
        steppers[num].setAcceleration(slowHomingAacceleration);
        steppers[num].setMaxSpeed(slowHomingSpeed);
        // Move a distance of backoffsteps OPPOSITE to homing direstion and check if the switch has been released
        steppers[num].move( -1 * homingDirection[num] * backoffSteps); //

        while (steppers[num].distanceToGo() != 0) {
          processEstop();
          if (eStop) { // check for estop, then run motor
            steppers[num].stop(); // CHECK: Does stop() also remove any distance left to go?
            error = 1;
            break; // Break the loop if switch was pressed. This way, error = 0 stays and enables other actions.
          } else { // no estop
            digitalWrite(TEST_LED_PIN, HIGH);
            steppers[num].run();
          }
        }

        digitalWrite(TEST_LED_PIN, HIGH);
        steppers[num].stop();
        delay(1); // 1ms delay - just in case

        // Check the limit switch at the end of each homing move
        // If endstop is still triggered after backoff
        // Funnily enough, just setting this and the other post-backoff check to LOW emulates a successful homing sequence
        if (digitalRead(limitSwitchPins[num]) == HIGH) {
          error = 1;
        }// if switch still pressed after 1st back-off
      } // if no error after second approach
    } // if no error after first backoff
  } // if no estop and no error after 1st homing switch approach

  // Return speed and acceleration back to normal regardless of errors
  steppers[num].setAcceleration(maxAacceleration);
  steppers[num].setMaxSpeed(maxSpeed);

  if (!eStop && !error) {
    // Write homedPosition to current and target position for the axis in question
    targetPositions[num] = homedPosition[num];
    currentPositions[num] = homedPosition[num];
    steppers[num].setCurrentPosition(homedPosition[num]); // Need to investigate if the output of this function of accelstepper is actually usable for this purpose.
    return true;
  }  else { // If an error was encountered
    Serial.print(gcLetters[num]);
    Serial.print(" axis ");
    Serial.println(ERR_HOMING_FAIL);
    return false;
  }
} // homeAxis
