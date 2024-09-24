/*
 * wallfollowing_with_avoid.c
 *
 *  Created on: Nov 12, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include "wallfollowing_with_avoid.h"

#include <math.h>
#include "usec_time.h"

#include "debug.h"

static float state_start_time;

//static variables only used for initialization
static bool first_run = true;
static float ref_distance_from_wall = 0.5;
static float max_speed = 0.5;
static float local_direction = 1;

static uint64_t wf3_start_time = 0;
static bool first_time_state_wf_3 = true;
static uint64_t wf3_current_time = 0;
static uint64_t wf3_duration = 0;
static uint64_t wf3_threshold = 1000 * 1000 * 18;


static int transition(int new_state)
{
    float t =  usecTimestamp() / 1e6;
    state_start_time = t;
    return new_state;
}

// statemachine functions
void init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall, float max_speed_ref,
        float starting_local_direction)
{
    ref_distance_from_wall = new_ref_distance_from_wall;
    max_speed = max_speed_ref;
    local_direction = starting_local_direction;
    first_run = true;
}


int wall_follower_and_avoid_controller(float *vel_x, float *vel_y, float *vel_w, float front_range, float left_range,
        float right_range,  float current_heading, uint8_t rssi_other_drone)
{

    // Initalize static variables
    static int state = 1;
    static int state_wf = 0;

    // if it is reinitialized
    if (first_run) {
        state = 1;
        float t =  usecTimestamp() / 1e6;
        state_start_time = t;
        first_run = false;
    }


    /***********************************************************
     * State definitions
     ***********************************************************/
    // 1 = forward
    // 2 = wall_following
    // 3 = move_out_of_way


    /***********************************************************
     * Handle state transitions
     ***********************************************************/
    if (state == 1) {     //FORWARD
        // if front range is close, start wallfollowing
        if (front_range < ref_distance_from_wall + drone_dist_from_wall_to_start_margin) {
            wall_follower_init(ref_distance_from_wall, max_speed, 3);
            state = transition(2); //wall_following
        }
    } else if (state == 2) {      //WALL_FOLLOWING
        if (rssi_other_drone < rssi_collision_threshold) {
            state = transition(3);
        }

        if(state_wf == 3) {
            if (first_time_state_wf_3) {
                wf3_start_time = usecTimestamp();
                first_time_state_wf_3 = false;
                DEBUG_PRINT("first_time_state_wf_3: %d ----------------------------------\n", first_time_state_wf_3);
            }
            wf3_current_time = usecTimestamp();
            wf3_duration = wf3_current_time - wf3_start_time;
            DEBUG_PRINT("wf3_start_time: %llu\n", wf3_start_time);
            DEBUG_PRINT("wf3_current_time: %llu\n", wf3_current_time);
            DEBUG_PRINT("wf3_duration: %llu\n", wf3_duration);
            if (wf3_duration > wf3_threshold) {

                DEBUG_PRINT("Time in State 3  > Threshold\n");
                if (current_heading >= 0 && current_heading <= 0.1f) {
                    state = transition(1);
                    first_time_state_wf_3 = true;
                }
            }
        }
        else {
            first_time_state_wf_3 = true;
        }
    } else if (state == 3) { //MOVE_OUT_OF_WAY
        if (rssi_other_drone > rssi_collision_threshold) {
            state = transition(1);
        }
    }

    /***********************************************************
     * Handle state actions
     ***********************************************************/

    float temp_vel_x = 0;
    float temp_vel_y = 0;
    float temp_vel_w = 0;

    if (state == 1) {        //FORWARD
        // forward max speed
        // DEBUG_PRINT("wallfollow_w_avoid: STATE 1 FORWARD\n");
        temp_vel_x = max_speed;

    } else  if (state == 2) {       //WALL_FOLLOWING
        //Get the values from the wallfollowing
        // DEBUG_PRINT("wallfollow_w_avoid: STATE 2 WALLFOLLOWING\n");
        if (local_direction == 1) {
            state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, local_direction);
        } else if (local_direction == -1) {
            state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, local_direction);
        }
    } else if (state == 3) {       //MOVE_OUT_OF_WAY

        // DEBUG_PRINT("wallfollow_w_avoid: STATE 3 MOVE OUT\n");

        float save_distance = 1.0f;
        if (left_range < save_distance) {
            temp_vel_y = temp_vel_y - 0.1f;
            // DEBUG_PRINT("move right\n");
        }
        if (right_range < save_distance) {
            temp_vel_y = temp_vel_y + 0.1f;
            // DEBUG_PRINT("move left\n");
        }
        if (front_range < save_distance) {
            temp_vel_x = temp_vel_x - 0.1f;
            // DEBUG_PRINT("move back\n");
        }

    }

    *vel_x = temp_vel_x;
    *vel_y = temp_vel_y;
    *vel_w = temp_vel_w;

    return state;
}

