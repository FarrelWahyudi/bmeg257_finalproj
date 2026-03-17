/*
 * ================================================================
 *  Posture Reminder Device - Algorithm Prototype
 *  Course: BMEG 257 Biomedical Engineering Design I
 *  Institution: University of British Columbia
 *
 *  Purpose:
 *  Prototype posture detection algorithm for a wearable posture
 *  reminder device. The system monitors trunk angle using IMU
 *  orientation data and triggers a vibration alert when sustained
 *  poor posture is detected.
 *
 *  The algorithm uses a finite state machine to classify posture
 *  into states such as GOOD_SITTING, BAD_POSTURE, or
 *  TRANSITION_MOVEMENT. Alerts are only triggered after posture
 *  deviation persists beyond a specified duration to reduce false
 *  notifications.
 *
 * ================================================================
 */


#include <math.h>


/* ================================================================
   Pin Definitions
   ================================================================ */
const int VIBRATION_PIN = 9;


/* ================================================================
   Algorithm Thresholds
   ================================================================ */
const float BAD_POSTURE_THRESHOLD = 15.0;
const float GOOD_POSTURE_THRESHOLD = 10.0;


const float GYRO_MOVEMENT_THRESHOLD = 20.0;
const float ACC_MOVEMENT_THRESHOLD = 12.0;


const unsigned long BAD_POSTURE_HOLD_TIME = 5000;
const unsigned long TRANSITION_IGNORE_TIME = 2500;
const unsigned long ALERT_COOLDOWN = 45000;
const unsigned long CALIBRATION_TIME = 4000;
const unsigned long SITTING_STABLE_TIME = 3000;


/* ================================================================
   IMU Variables
   ================================================================ */
float pitch_filt = 0.0;
float gyro_pitch = 0.0;


float ax = 0.0;
float ay = 0.0;
float az = 0.0;


float acc_mag = 0.0;


float pitch_baseline = 0.0;
float delta_pitch = 0.0;


/* ================================================================
   Timing Variables
   ================================================================ */


unsigned long bad_start_time = 0;
unsigned long transition_start_time = 0;
unsigned long last_alert_time = 0;
unsigned long sitting_stable_start_time = 0;


/* ================================================================
   State Machine Definition
   ================================================================ */


enum PostureState {
    GOOD_SITTING,
    BAD_CANDIDATE_FORWARD,
    BAD_CANDIDATE_BACKWARD,
    BAD_CONFIRMED_FORWARD,
    BAD_CONFIRMED_BACKWARD,
    TRANSITION_MOVEMENT,
    NOT_SITTING
};


PostureState currentState = GOOD_SITTING;


/* ================================================================
   Sensor Reading (Simulated Data)
   ================================================================ */


void readIMU() {
    /* Simulated sensor values for prototype testing */
    pitch_filt = 25.0;
    gyro_pitch = 1.0;


    ax = 0;
    ay = 0;
    az = 9.8;


    acc_mag = sqrt(ax * ax + ay * ay + az * az);
}


/* ================================================================
   Movement Detection
   ================================================================ */
bool isFastMovement() {
    return (fabs(gyro_pitch) > GYRO_MOVEMENT_THRESHOLD) ||
           (acc_mag > ACC_MOVEMENT_THRESHOLD);
}


bool isNotSittingPattern() {
    if (isFastMovement() && fabs(delta_pitch) < GOOD_POSTURE_THRESHOLD) {
        return true;
    }
    return false;
}


/* ================================================================
   Alert Control
   ================================================================ */


bool canTriggerAlert(unsigned long current_time) {
    return (current_time - last_alert_time) >= ALERT_COOLDOWN;
}


void triggerVibration() {
    digitalWrite(VIBRATION_PIN, HIGH);
    delay(500);
    digitalWrite(VIBRATION_PIN, LOW);
}


/* ================================================================
   Baseline Calibration
   ================================================================ */


void calibrateBaseline() {
    unsigned long start_time = millis();
    float pitch_sum = 0.0;
    int sample_count = 0;


    while (millis() - start_time < CALIBRATION_TIME) {
        readIMU();
        pitch_sum += pitch_filt;
        sample_count++;
        delay(20);
    }


    if (sample_count > 0) {
        pitch_baseline = pitch_sum / sample_count;
    }
}


/* ================================================================
   Setup
   ================================================================ */
void setup() {
    Serial.begin(9600);
    pinMode(VIBRATION_PIN, OUTPUT);
    digitalWrite(VIBRATION_PIN, LOW);
    calibrateBaseline();
    pitch_baseline = 5.0;
    currentState = GOOD_SITTING;
    last_alert_time = 0;
}


/* ================================================================
   Main Control Loop
   ================================================================ */


void loop() {
    unsigned long current_time = millis();
    readIMU();
    delta_pitch = pitch_filt - pitch_baseline;
    switch (currentState) {
        case GOOD_SITTING:
            if (isNotSittingPattern()) {
                currentState = NOT_SITTING;
                sitting_stable_start_time = 0;
            } else if (delta_pitch >= BAD_POSTURE_THRESHOLD) {
                if (isFastMovement()) {
                    currentState = TRANSITION_MOVEMENT;
                    transition_start_time = current_time;
                } else {
                    currentState = BAD_CANDIDATE_FORWARD;
                    bad_start_time = current_time;
                }
            } else if (delta_pitch <= -BAD_POSTURE_THRESHOLD) {
                if (isFastMovement()) {
                    currentState = TRANSITION_MOVEMENT;
                    transition_start_time = current_time;
                } else {
                    currentState = BAD_CANDIDATE_BACKWARD;
                    bad_start_time = current_time;
                }
            }
            break;


        case BAD_CANDIDATE_FORWARD:
            if (fabs(delta_pitch) <= GOOD_POSTURE_THRESHOLD) {
                currentState = GOOD_SITTING;
            } else if (isFastMovement()) {
                currentState = TRANSITION_MOVEMENT;
                transition_start_time = current_time;


            } else if (delta_pitch >= BAD_POSTURE_THRESHOLD &&
                       current_time - bad_start_time >= BAD_POSTURE_HOLD_TIME) {
                currentState = BAD_CONFIRMED_FORWARD;
                if (canTriggerAlert(current_time)) {
                    triggerVibration();
                    last_alert_time = current_time;
                }
            }
            break;


        case BAD_CANDIDATE_BACKWARD:
            if (fabs(delta_pitch) <= GOOD_POSTURE_THRESHOLD) {
                currentState = GOOD_SITTING;
            } else if (isFastMovement()) {
                currentState = TRANSITION_MOVEMENT;
                transition_start_time = current_time;
            } else if (delta_pitch <= -BAD_POSTURE_THRESHOLD &&
                       current_time - bad_start_time >= BAD_POSTURE_HOLD_TIME) {
                currentState = BAD_CONFIRMED_BACKWARD;
                if (canTriggerAlert(current_time)) {
                    triggerVibration();
                    last_alert_time = current_time;
                }
            }
            break;
        case BAD_CONFIRMED_FORWARD:
        case BAD_CONFIRMED_BACKWARD:
            if (fabs(delta_pitch) <= GOOD_POSTURE_THRESHOLD) {
                currentState = GOOD_SITTING;
            } else if (isFastMovement()) {
                currentState = TRANSITION_MOVEMENT;
                transition_start_time = current_time;
            } else {
                if (canTriggerAlert(current_time)) {
                    triggerVibration();
                    last_alert_time = current_time;
                }
            }
            break;


        case TRANSITION_MOVEMENT:
            if (current_time - transition_start_time >= TRANSITION_IGNORE_TIME) {
                if (isNotSittingPattern()) {
                    currentState = NOT_SITTING;
                } else if (fabs(delta_pitch) <= GOOD_POSTURE_THRESHOLD) {
                    currentState = GOOD_SITTING;
                } else if (delta_pitch >= BAD_POSTURE_THRESHOLD) {
                    currentState = BAD_CANDIDATE_FORWARD;
                    bad_start_time = current_time;
                } else if (delta_pitch <= -BAD_POSTURE_THRESHOLD) {
                    currentState = BAD_CANDIDATE_BACKWARD;
                    bad_start_time = current_time;
                } else {
                    currentState = GOOD_SITTING;
                }
            }


            break;


        case NOT_SITTING:
            if (!isFastMovement() &&
                fabs(delta_pitch) <= GOOD_POSTURE_THRESHOLD) {
                if (sitting_stable_start_time == 0) {
                    sitting_stable_start_time = current_time;
                }
                if (current_time - sitting_stable_start_time >=
                    SITTING_STABLE_TIME) {
                    currentState = GOOD_SITTING;
                    sitting_stable_start_time = 0;
                }


            } else {
                sitting_stable_start_time = 0;
            }
            break;
    }


    Serial.print("delta_pitch: ");
    Serial.print(delta_pitch);
    Serial.print(" | state: ");
    Serial.println(currentState);


    delay(50);
}
