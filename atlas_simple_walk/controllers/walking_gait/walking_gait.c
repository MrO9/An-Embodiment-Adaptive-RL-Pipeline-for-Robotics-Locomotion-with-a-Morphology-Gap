#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16
#define STEP_TIME 1.5  
#define STEP_HEIGHT 0.035 
#define STEP_LENGTH 0.09  // FIXED: Will be flipped to move forward
#define ARM_POSITION -1.2
#define INVERSE_ARM_POSITION 1.2

// **Tuned Balance Control Gains**
#define KP_HIP 0.25  // Increased for more stability
#define KD_HIP 0.12  
#define KP_ANKLE 0.2  
#define KD_ANKLE 0.1  

#define MAX_CORRECTION 0.05  

#define FALLBACK_THRESHOLD -2.5  

WbDeviceTag gyro, accelerometer, gps;

void initialize_sensors() {
    gyro = wb_robot_get_device("gyro");
    accelerometer = wb_robot_get_device("accelerometer");
    gps = wb_robot_get_device("gps");

    if (!gyro || !accelerometer || !gps) {
        printf("Error: Sensors not found!\n");
        wb_robot_cleanup();
        exit(EXIT_FAILURE);
    }

    wb_gyro_enable(gyro, TIME_STEP);
    wb_accelerometer_enable(accelerometer, TIME_STEP);
    wb_gps_enable(gps, TIME_STEP);
}

void safe_step() {
    if (wb_robot_step(TIME_STEP) == -1) {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

void set_arms_position(WbDeviceTag left_arm, WbDeviceTag right_arm) {
    wb_motor_set_position(left_arm, ARM_POSITION);
    wb_motor_set_position(right_arm, INVERSE_ARM_POSITION);
    for (int i = 0; i < 50; i++) safe_step();
}

void compute_balance_corrections(double *correction_hip, double *correction_ankle) {
    const double *gyro_values = wb_gyro_get_values(gyro);
    const double *accel_values = wb_accelerometer_get_values(accelerometer);

    double roll_rate = gyro_values[0];  
    double accel_pitch = accel_values[1];

    *correction_hip = KP_HIP * accel_pitch + KD_HIP * roll_rate;
    *correction_ankle = KP_ANKLE * accel_pitch + KD_ANKLE * roll_rate;

    if (*correction_hip > MAX_CORRECTION) *correction_hip = MAX_CORRECTION;
    if (*correction_hip < -MAX_CORRECTION) *correction_hip = -MAX_CORRECTION;
    if (*correction_ankle > MAX_CORRECTION) *correction_ankle = MAX_CORRECTION;
    if (*correction_ankle < -MAX_CORRECTION) *correction_ankle = -MAX_CORRECTION;
}

// **Fallback Stance (if falling backward)**
void fallback_stance(WbDeviceTag l_knee, WbDeviceTag r_knee, WbDeviceTag l_ankle, WbDeviceTag r_ankle, WbDeviceTag l_hip, WbDeviceTag r_hip, WbDeviceTag l_arm, WbDeviceTag r_arm) {
    printf("⚠️ FALLBACK ACTIVATED! ⚠️\n");

    wb_motor_set_position(l_knee, 0.3);
    wb_motor_set_position(r_knee, 0.3);

    wb_motor_set_position(l_arm, -0.8);
    wb_motor_set_position(r_arm, 0.8);

    wb_motor_set_position(l_hip, -0.2);
    wb_motor_set_position(r_hip, 0.2);

    for (int i = 0; i < 30; i++) safe_step();
}

// **Walking Pattern (NOW MOVES FORWARD)**
void generate_walking_pattern(WbDeviceTag l_hip, WbDeviceTag r_hip, WbDeviceTag l_knee, WbDeviceTag r_knee, WbDeviceTag l_ankle, WbDeviceTag r_ankle, WbDeviceTag l_arm, WbDeviceTag r_arm) {
    double t = 0.0;
    double T = STEP_TIME * 2;

    while (t < T) {
        const double *accel_values = wb_accelerometer_get_values(accelerometer);
        double accel_pitch = accel_values[1];

        if (accel_pitch < FALLBACK_THRESHOLD) {
            fallback_stance(l_knee, r_knee, l_ankle, r_ankle, l_hip, r_hip, l_arm, r_arm);
        }

        double phase = t / T;
        double weight_shift = 0.015 * sin(2 * M_PI * phase);

        // **FLIPPED SIGNS: Now moves FORWARD**
        double left_leg_pos = (-STEP_LENGTH * sin(M_PI * phase) * 0.8) + weight_shift;
        double right_leg_pos = (-STEP_LENGTH * sin(M_PI * (phase + 0.5)) * 0.8) - weight_shift;

        // **Ensure knees don't go negative**
        double left_knee_pos = fmax(STEP_HEIGHT * sin(2 * M_PI * phase) * 0.6, 0.0);
        double right_knee_pos = fmax(STEP_HEIGHT * sin(2 * M_PI * (phase + 0.5)) * 0.6, 0.0);

        double correction_hip, correction_ankle;
        compute_balance_corrections(&correction_hip, &correction_ankle);

        wb_motor_set_position(l_hip, left_leg_pos + correction_hip);
        wb_motor_set_position(r_hip, right_leg_pos - correction_hip);
        wb_motor_set_position(l_knee, left_knee_pos);
        wb_motor_set_position(r_knee, right_knee_pos);
        wb_motor_set_position(l_ankle, -left_leg_pos * 0.5 + correction_ankle);
        wb_motor_set_position(r_ankle, -right_leg_pos * 0.5 - correction_ankle);

        t += TIME_STEP / 1000.0;
        safe_step();
    }
}

int main() {
    wb_robot_init();
    initialize_sensors();

    WbDeviceTag l_hip = wb_robot_get_device("LLegLhy");
    WbDeviceTag r_hip = wb_robot_get_device("RLegLhy");
    WbDeviceTag l_knee = wb_robot_get_device("LLegKny");
    WbDeviceTag r_knee = wb_robot_get_device("RLegKny");
    WbDeviceTag l_ankle = wb_robot_get_device("LLegUay");
    WbDeviceTag r_ankle = wb_robot_get_device("RLegUay");
    WbDeviceTag l_arm = wb_robot_get_device("LArmShx");
    WbDeviceTag r_arm = wb_robot_get_device("RArmShx");

    printf("Starting balanced walking (FORWARD)...\n");
    set_arms_position(l_arm, r_arm);

    while (true) {
        generate_walking_pattern(l_hip, r_hip, l_knee, r_knee, l_ankle, r_ankle, l_arm, r_arm);
    }

    wb_robot_cleanup();
    return EXIT_SUCCESS;
}
