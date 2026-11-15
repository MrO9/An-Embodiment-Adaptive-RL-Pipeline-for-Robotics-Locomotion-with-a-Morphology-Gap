#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16
#define STEP_TIME 1.8      // Slower stepping rhythm
#define STEP_HEIGHT 0.025  // Lower step height
#define STEP_LENGTH 0.06   // Reduced step length
#define TORSO_LEAN 0.05    // Forward torso leaning

// Improved Balance Control Gains
#define KP_HIP 0.3
#define KD_HIP 0.15
#define KP_ANKLE 0.35      // Stronger ankle correction
#define KD_ANKLE 0.2

#define MAX_CORRECTION 0.07
#define FALLBACK_THRESHOLD -2.0

WbDeviceTag gyro, accelerometer, gps;

void initialize_sensors() {
    gyro = wb_robot_get_device("gyro");
    accelerometer = wb_robot_get_device("accelerometer");
    gps = wb_robot_get_device("gps");

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

void compute_balance_corrections(double *hip_corr, double *ankle_corr) {
    const double *gyro_data = wb_gyro_get_values(gyro);
    const double *accel_data = wb_accelerometer_get_values(accelerometer);
    
    // Use combined pitch estimation
    double pitch_rate = gyro_data[0];
    double accel_pitch = atan2(accel_data[0], sqrt(accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]));
    
    *hip_corr = KP_HIP * accel_pitch + KD_HIP * pitch_rate;
    *ankle_corr = KP_ANKLE * accel_pitch + KD_ANKLE * pitch_rate;
    
    // Clamp corrections
    *hip_corr = fmin(fmax(*hip_corr, -MAX_CORRECTION), MAX_CORRECTION);
    *ankle_corr = fmin(fmax(*ankle_corr, -MAX_CORRECTION), MAX_CORRECTION);
}

void walking_phase(WbDeviceTag *motors, double phase, double hip_corr, double ankle_corr) {
    double t = fmod(phase, 1.0);
    double swing = sin(M_PI * t);
    double lift = sin(2 * M_PI * t);
    
    // Left leg parameters
    double left_hip = -STEP_LENGTH * swing;
    double left_knee = fmax(STEP_HEIGHT * lift, 0.0);
    double left_ankle = STEP_LENGTH * 0.4 * swing - TORSO_LEAN;
    
    // Right leg parameters (180° out of phase)
    double right_hip = -STEP_LENGTH * sin(M_PI * (t + 0.5));
    double right_knee = fmax(STEP_HEIGHT * sin(2 * M_PI * (t + 0.5)), 0.0);
    double right_ankle = STEP_LENGTH * 0.4 * sin(M_PI * (t + 0.5)) - TORSO_LEAN;

    // Apply balance corrections
    wb_motor_set_position(motors[0], left_hip + hip_corr);     // L_Hip
    wb_motor_set_position(motors[1], right_hip - hip_corr);    // R_Hip
    wb_motor_set_position(motors[2], left_knee);               // L_Knee
    wb_motor_set_position(motors[3], right_knee);              // R_Knee
    wb_motor_set_position(motors[4], left_ankle + ankle_corr); // L_Ankle
    wb_motor_set_position(motors[5], right_ankle - ankle_corr);// R_Ankle
}

int main() {
    wb_robot_init();
    initialize_sensors();

    // Motor initialization
    WbDeviceTag motors[6] = {
        wb_robot_get_device("LLegLhy"),  // Left Hip
        wb_robot_get_device("RLegLhy"),  // Right Hip
        wb_robot_get_device("LLegKny"),  // Left Knee
        wb_robot_get_device("RLegKny"),  // Right Knee
        wb_robot_get_device("LLegUay"),  // Left Ankle
        wb_robot_get_device("RLegUay")   // Right Ankle
    };

    // Arm motors for balance
    WbDeviceTag l_arm = wb_robot_get_device("LArmShx");
    WbDeviceTag r_arm = wb_robot_get_device("RArmShx");
    wb_motor_set_position(l_arm, -1.2);
    wb_motor_set_position(r_arm, 1.2);

    printf("Starting stabilized walking...\n");
    double phase = 0.0;
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // Balance computations
        double hip_corr, ankle_corr;
        compute_balance_corrections(&hip_corr, &ankle_corr);
 
        // Walking pattern generation
        walking_phase(motors, phase, hip_corr, ankle_corr);
        
        // Arm swing for balance
        //wb_motor_set_position(l_arm, -0.5 * sin(2 * M_PI * phase));
        //wb_motor_set_position(r_arm, 0.5 * sin(2 * M_PI * phase));
        
        phase += TIME_STEP / (STEP_TIME * 1000);
    }

    wb_robot_cleanup();
    return 0;
}