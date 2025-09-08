/**
 * Control loop abstractions.
 */

#pragma once

#include "stm32f4xx_hal.h"

#define MAX_VELOCITY 10.00     // Max linear velocity in cm/s found from testing
#define MAX_ACCELERATION 3.00  // Max acceleration in cm/s^2 found from testing
#define WHEEL_RADIUS 1.215     // Example value in centimeters
#define WHEEL_BASE 8.7       // Example value in centimeters
#define CPR 358.32              // Encoder counts per revolution
#define ENCODER_MAX_COUNT UINT16_MAX

// Structure to store PID parameters and state
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float min_output;
    float max_output;
    float prev_error;
    float derf;
} PIDController;


typedef struct {
    float position;
    float velocity;
    float acceleration;
} State;

typedef struct {
    float v_max;
    float a_max;
    float distance;
    float t1;
    float t2;
    float t3;
    float total_time;
    float OGVelocity;
} TrapezoidalProfile;

extern uint16_t encoder_counter_R;
extern uint16_t encoder_counter_L;
extern Motor rmotor;
extern Motor lmotor;

extern uint16_t ir_data[3];
extern float output_L;
extern float output_R;
extern float toleranceL;
extern float toleranceR ;
extern float left_distance, right_distance;
extern float leftVelocity, rightVelocity, angularVelocity;

extern float velocityL ;
extern float velocityR;

extern float elapsedTime;


void init_trap_traj( TrapezoidalProfile *traj, float V_max, float A_max, float target_dist) {
    traj->v_max = V_max;
    traj->a_max = A_max;
    traj->distance = target_dist;
}

void init_pid( PIDController *pid, float kp, float ki, float kd, float min_output, float max_output) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0;
    pid->min_output = min_output;
    pid->max_output = max_output;
    pid->prev_error = 0.0;
    pid->derf =0;
}

// Compute PID output
float compute_pid(PIDController *pid, float setpoint, float actual, float buff) {
    float error = setpoint - actual;
    pid->integral += error ;
    if (pid->integral > pid->max_output / pid->Ki) {
        pid->integral = pid->max_output / pid->Ki;
    } else if (pid->integral < pid->min_output / pid->Ki) {
        pid->integral = pid->min_output / pid->Ki;
    }

    float derivative = (error - pid->prev_error);
    float derfil = pid->derf * 0.8 + derivative *.2;
    pid->derf = derfil;
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derfil);
    if((output > 0 )&& (output < buff )) {
    	output = output + buff;
    }
    if((output < 0 )&& (output > -buff )) {
    	output = output - buff;
    }
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }
    pid->prev_error = error;
    return output;
}

void calculate_trapezoidal_profile(TrapezoidalProfile *profile) {
    profile->t1 = profile->v_max / profile->a_max;
    float d1 = 0.5 * profile->a_max * profile->t1 * profile->t1;

    if (fabs(profile->distance) < 2 * d1) {
        // Adjust for triangular profile if distance is too short
        profile->t1 = sqrt(fabs(profile->distance) / profile->a_max);
        profile->t2 = 0;
        profile->t3 = profile->t1;
        profile->total_time = 2 * profile->t1;
    } else {
        profile->t2 = (profile->distance - 2 * d1) / profile->v_max;
        profile->t3 = profile->t1;
        profile->total_time = profile->t1 + profile->t2 + profile->t3;
    }
}

float get_velocity(TrapezoidalProfile *profile, float t, float currentVelocity) {
    if (profile->distance > 0 ){
	if (t < profile->t1) {
        // Acceleration phase
    	profile->OGVelocity = currentVelocity;
        return profile->a_max * t;
    } else if (t < (profile->t1 + profile->t2)) {
        // Constant velocity phase
        return profile->v_max;
    } else if (t < profile->total_time) {
        // Deceleration phase
    	if (profile->t2 == 0){
    		return profile->OGVelocity - profile->a_max*(t - profile->t1);}
    	else{
        return profile->v_max - profile->a_max * (t - profile->t1 - profile->t2);}
    }
     else {
        // Stop
        return 0;
    }
    }else{
    if (t < profile->t1) {
        // Acceleration phase
    	profile->OGVelocity = currentVelocity;
        return (-profile->a_max * t);
    } else if (t < (profile->t1 + profile->t2)) {
        // Constant velocity phase
        return (-profile->v_max);
    } else if (t < profile->total_time) {
        // Deceleration phase
    	if (profile->t2 == 0){
    		return (profile->OGVelocity + profile->a_max*(t - profile->t1));}
    	else{
        return (-profile->v_max + profile->a_max * (t - profile->t1 - profile->t2));}
    }
     else {
        // Stop
        return 0;
    }
    }
}

void go_distance(float desired_dist_L, float desired_dist_R, uint8_t needWcontrol){
	PIDController pidL;
	PIDController pidR;
	PIDController pidW_velocity;
	PIDController wallPID;
	TrapezoidalProfile profileL;
	TrapezoidalProfile profileR;
	init_trap_traj(&profileL, 50.0, 150, desired_dist_L);
	init_trap_traj(&profileR, 50.0, 150, desired_dist_R);
	calculate_trapezoidal_profile(&profileL) ;
	calculate_trapezoidal_profile(&profileR) ;
	init_pid(&pidL, .0015, .0005, .09, -.8, .8);  // Tune these parameters
	init_pid(&pidR, .0015, .0005, .09, -.82 ,.82);  // Tune these parameters
	init_pid(&pidW_velocity, .01, .001, .01, -.2, .2);  // Tune these parameters
	init_pid(&wallPID, .0001, .00005, .0005, -.2, .2);  // Tune these parameters
	uint16_t start_encoder_count_L = TIM3->CNT;
	uint16_t start_encoder_count_R = TIM4->CNT;
	elapsedTime = 0;
	float prev_elapsedTime = 0;
	float prev_distL = 0;
	float prev_distR = 0;
	float tolerancecompareL = .07;
	float tolerancecompareR = .07;
	float prev_velocityL = 0;
	float prev_velocityR = 0;
	float wallcorrect = 0;
	toleranceL = 100;
	toleranceR = 100;

	while((toleranceL > tolerancecompareL )|| (toleranceL < -tolerancecompareL)||(toleranceR > tolerancecompareR) || (toleranceR < -tolerancecompareR)){
		if (elapsedTime != prev_elapsedTime) {
			int16_t delta_count_L = (int16_t)encoder_counter_L - (int16_t)start_encoder_count_L ;
			int16_t delta_count_R = (int16_t)encoder_counter_R - (int16_t)start_encoder_count_R ;
			// Handle overflow
			if (delta_count_L <= -ENCODER_MAX_COUNT / 2) {
				delta_count_L += ENCODER_MAX_COUNT + 1;
			} else if (delta_count_L >= ENCODER_MAX_COUNT / 2) {
				delta_count_L -= ENCODER_MAX_COUNT - 1;
			}
			if (delta_count_R <= -ENCODER_MAX_COUNT / 2) {
				delta_count_R += ENCODER_MAX_COUNT + 1;
			} else if (delta_count_R >= ENCODER_MAX_COUNT / 2) {
				delta_count_R -= ENCODER_MAX_COUNT - 1;
			}
			left_distance  =  delta_count_L * (2 * M_PI * WHEEL_RADIUS) / CPR;
			right_distance =  delta_count_R * (2 * M_PI * WHEEL_RADIUS) / CPR;
			toleranceL = desired_dist_L - left_distance ;
			toleranceR =  desired_dist_R- right_distance;
			float deltadistL = left_distance - prev_distL;
			float deltadistR = right_distance - prev_distR;
			prev_distL = left_distance;
			prev_distR = right_distance;
			float deltaElapsedTime = elapsedTime - prev_elapsedTime ;
			prev_elapsedTime = elapsedTime;
			// Convert position to encoder counts
		    leftVelocity = deltadistL / deltaElapsedTime ;// cm/s
		    rightVelocity = deltadistR / deltaElapsedTime ;
			float filteredVelocityL = (prev_velocityL * .8 ) + (leftVelocity * .2);
			float filteredVelocityR = (prev_velocityR * .8 ) + (rightVelocity * .2);
			angularVelocity = filteredVelocityL - filteredVelocityR;
			float W_control = compute_pid(&pidW_velocity, 0, angularVelocity, 0);
			prev_velocityL = filteredVelocityL;
			prev_velocityR = filteredVelocityR;
//		    if (threetimes == 3){
			velocityL = get_velocity(&profileL, elapsedTime, velocityL);
			velocityR = get_velocity(&profileR, elapsedTime, velocityR);
			//wall correction
			if (needWcontrol == 1){
			      if(ir_data[0] > 800){
			    	  wallcorrect = compute_pid(&wallPID, 800, ir_data[0], 0);
			    	  output_L = (compute_pid(&pidL, velocityL, filteredVelocityL, .6))- wallcorrect ;
			    	  output_R = (compute_pid(&pidR, velocityR, filteredVelocityR, .53))+ wallcorrect;
			      }
			      else if(ir_data[2] > 800){
			    	  wallcorrect = compute_pid(&wallPID, 800, ir_data[2], 0);
			    	  output_L = (compute_pid(&pidL, velocityL, filteredVelocityL, .53))+ wallcorrect ;
			    	  output_R = (compute_pid(&pidR, velocityR, filteredVelocityR, .6))- wallcorrect;
			      }
			      else {
			    	  wallcorrect = 0;
						output_L = (compute_pid(&pidL, velocityL, filteredVelocityL, .57)) ;
						output_R = (compute_pid(&pidR, velocityR, filteredVelocityR, .57)) ;
						if (needWcontrol == 1){
							output_L += W_control ;
							output_R -= W_control;
						}
			      }

			}
			else{
				output_L = (compute_pid(&pidL, velocityL, filteredVelocityL, .57)) ;
				output_R = (compute_pid(&pidR, velocityR, filteredVelocityR, .57)) ;
			}

//			drive_motor(&lmotor, (output_L));
//			drive_motor(&rmotor, output_R);

//			drive_motor(&lmotor, .6);
//			drive_motor(&rmotor, .6);
			if (elapsedTime > profileL.total_time){
//				output_R = compute_pid(&pidR, desired_dist_R , right_distance, .48);
//				output_L = compute_pid(&pidL, desired_dist_L , left_distance, .43);
				drive_motor(&lmotor, .7);
				drive_motor(&rmotor, .7);
				return 2;
			}else{
//				output_L = compute_pid(&pidL, velocityL, filteredVelocityL, .55);
//				output_R = compute_pid(&pidR, velocityR, filteredVelocityR, .5);
			drive_motor(&lmotor, (output_L));
			drive_motor(&rmotor, output_R);
			}
			// Get current velocities

			}

		}
	drive_motor(&lmotor, 0);
    drive_motor(&rmotor, 0);
	}
