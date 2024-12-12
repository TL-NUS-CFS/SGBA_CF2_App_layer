/*
 * gradient_bug.c
 *
 *  Created on: Aug 9, 2018
 *      Author: knmcguire
 */


#include <string.h>
#include <errno.h>
#define __USE_MISC
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "commander.h"
#include "sensors.h"
#include "stabilizer_types.h"

#include "estimator_kalman.h"
#include "stabilizer.h"

#include "wallfollowing_multiranger_onboard.h"
#include "wallfollowing_with_avoid.h"
#include "SGBA.h"
#include "usec_time.h"

#include "range.h"
#include "radiolink.h"
#include "median_filter.h"
#include "configblock.h"
#include "debug.h"

#define DEBUG_MODULE "SGBA"

#define STATE_MACHINE_COMMANDER_PRI 3

static bool keep_flying = false;
static bool is_flying = false;
static bool move_away_from_walls = false;
const float MIN_DISTANCE = 1.0; // m

float height;

static bool taken_off = false;

// Switch to multiple methods, that increases in complexity 
//1= wall_following: Go forward and follow walls with the multiranger 
//2=wall following with avoid: This also follows walls but will move away if another crazyflie with an lower ID is coming close, 
//3=SGBA: The SGBA method that incorperates the above methods.
//        NOTE: the switching between outbound and inbound has not been implemented yet
#define METHOD 2


void p2pcallbackHandler(P2PPacket *p);
static uint8_t rssi_inter;
static uint8_t rssi_inter_filtered;
static uint8_t rssi_inter_closest;

float rssi_angle_inter_ext;
float rssi_angle_inter_closest;
static uint8_t rssi_beacon;
static uint8_t rssi_beacon_filtered;

static uint8_t id_inter_ext;
static setpoint_t setpoint_BG;
static float vel_x_cmd, vel_y_cmd, vel_w_cmd;
static float heading_rad;
static float right_range;
static float front_range;
static float left_range;
static float up_range;
static float back_range;
static float rssi_angle;
static int state;


#if METHOD == 3
static int state_wf;
#endif
static float up_range_filtered;
static logVarId_t varid;
static paramVarId_t paramid;
//static bool manual_startup = false;
static bool on_the_ground = true;
//static uint32_t time_stamp_manual_startup_command = 0;
static bool correctly_initialized;
// static uint8_t rssi_array_other_drones[20] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
static uint8_t rssi_array_other_drones[40] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
static uint64_t time_array_other_drones[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static float rssi_angle_array_other_drones[40] = {500.0f};
static uint8_t id_inter_closest=100;

static struct MedianFilterFloat medFiltDrones[40];

#define MANUAL_STARTUP_TIMEOUT  M2T(3000)



static void take_off(setpoint_t *sp, float velocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = velocity;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void land(setpoint_t *sp, float velocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = - velocity;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}


static void hover(setpoint_t *sp, float height)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeAbs;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->position.z = height;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void vel_command(setpoint_t *sp, float vel_x, float vel_y, float yaw_rate, float height)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeAbs;
  sp->velocity.x = vel_x;
  sp->velocity.y = vel_y;
  sp->position.z = height;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = yaw_rate;
  sp->velocity_body = true;

}

static void shut_off_engines(setpoint_t *sp)
{
  sp->mode.x = modeDisable;
  sp->mode.y = modeDisable;
  sp->mode.z = modeDisable;
  sp->mode.yaw = modeDisable;

}

static int32_t find_minimum(uint8_t a[], int32_t n)
{
  int32_t c, min, index;

  min = a[0];
  index = 0;

  for (c = 1; c < n; c++) {
    if (a[c] < min) {
      index = c;
      min = a[c];
    }
  }

  return index;
}

/*static double wraptopi(double number)
{

  if(number>(double)M_PI)
    return (number-(double)(2*M_PI));
  else if(number< (double)(-1*M_PI))
    return (number+(double)(2*M_PI));
  else
    return (number);

}*/

bool is_close(float range) {
  return range < MIN_DISTANCE; // still too close to wall 
}

void appMain(void *param)
{
  static struct MedianFilterFloat medFilt;
  init_median_filter_f(&medFilt, 5);
  static struct MedianFilterFloat medFilt_2;
  init_median_filter_f(&medFilt_2, 5);
  static struct MedianFilterFloat medFilt_3;
  init_median_filter_f(&medFilt_3, 13);

  //Init filters for each drone
  for (uint8_t i = 0; i < 40; i++)
    init_median_filter_f(&medFiltDrones[i], 5);

  p2pRegisterCB(p2pcallbackHandler);
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
  static P2PPacket p_reply;
  p_reply.port=0x00;
  p_reply.data[0]=my_id;
  memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));
  p_reply.size=5;
  //DEBUG_PRINT("appMain");

#if METHOD!=1
  static uint64_t radioSendBroadcastTime=0;
#endif

  static uint64_t takeoffdelaytime = 0;

  #if METHOD==3
  static bool outbound = true;
  #endif

  systemWaitStart();
  vTaskDelay(M2T(3000));
  while (1) {
    // DEBUG_PRINT("state_machine: address = %llu\n", address);
    // DEBUG_PRINT("state_machine: my_id = %i\n", my_id);
	// some delay before the whole thing starts
    vTaskDelay(10);

    // For every 1 second (depend on rssi_reset_interval), reset the RSSI value to high if it hasn't been received for a while
    for (uint8_t it = 0; it < 40; it++)
    {
      uint64_t currentTimestamp = usecTimestamp();
      uint64_t otherDroneTimestamp = time_array_other_drones[it];
      // uint64_t deltaTime = currentTimestamp - otherDroneTimestamp;
      const uint64_t cutoffTime = 1000*1000*rssi_reset_interval;

      //if (usecTimestamp() >= time_array_other_drones[it] + 1000*1000*3) {
      if (currentTimestamp >= otherDroneTimestamp + cutoffTime) {
      
        time_array_other_drones[it] = currentTimestamp + cutoffTime+1;
        rssi_array_other_drones[it] = 150;
        rssi_angle_array_other_drones[it] = 500.0f;

        init_median_filter_f(&medFiltDrones[it], 5);
        // DEBUG_PRINT("resetting RSSI for drone %i and delta is %lld\n", it, deltaTime);
      }
    }

    // get RSSI, id and angle of closests crazyflie.
    id_inter_closest = (uint8_t)find_minimum(rssi_array_other_drones, 40);
    rssi_inter_closest = rssi_array_other_drones[id_inter_closest];
    rssi_angle_inter_closest = rssi_angle_array_other_drones[id_inter_closest];


    // filter rssi
    /*static int pos_avg = 0;
    static long sum = 0;
    static int arrNumbers[76] = {35};
    static int len = sizeof(arrNumbers) / sizeof(int);
    rssi_beacon_filtered = (uint8_t)movingAvg(arrNumbers, &sum, pos_avg, len, (int)rssi_beacon);*/


    /*static int arrNumbers_inter[10] = {35};
    static int len_inter = 10;//sizeof(arrNumbers_inter) / sizeof(int);
    static int pos_avg_inter = 0;
    static long sum_inter = 0;
    rssi_inter_filtered = (uint8_t)movingAvg(arrNumbers_inter, &sum_inter, pos_avg_inter, len_inter, (int)rssi_inter_closest);*/
    rssi_inter_filtered =  (uint8_t)update_median_filter_f(&medFilt_2, (float)rssi_inter_closest);


    /*  // FOR RSSI CA DEBUGGING //
    DEBUG_PRINT("state_machine: my_id = %i\n", my_id);
    DEBUG_PRINT("state_machine: rssi array = ");
    int loop;
    for (loop=0; loop<40; loop++) {
      DEBUG_PRINT("%d ", rssi_array_other_drones[loop]);
    }
    DEBUG_PRINT("state_machine: rssi of closest = %i\n", (int)rssi_array_other_drones[id_inter_closest]);
    DEBUG_PRINT("state_machine: rssi_inter_filtered = %i\n", (int)rssi_inter_filtered);*/


    //checking init of multiranger and flowdeck
    paramid = paramGetVarId("deck", "bcMultiranger");
    uint8_t multiranger_isinit=paramGetInt(paramid);
    paramid = paramGetVarId("deck", "bcFlow2");
    uint8_t flowdeck_isinit=paramGetUint(paramid);

    // get current height and heading
    varid = logGetVarId("kalman", "stateZ");
    height = logGetFloat(varid);
    varid = logGetVarId("stabilizer", "yaw");
    float heading_deg = logGetFloat(varid);
    heading_rad = heading_deg * (float)M_PI / 180.0f;

    //t RSSI of beacon
    varid = logGetVarId("radio", "rssi");
    rssi_beacon = logGetFloat(varid);
    rssi_beacon_filtered =  (uint8_t)update_median_filter_f(&medFilt_3, (float)rssi_beacon);


    /* filter rssi
    static int pos_avg = 0;
    static long sum = 0;
    static int arrNumbers[26] = {35};
    static int len = sizeof(arrNumbers) / sizeof(int);
    rssi_beacon_filtered = (uint8_t)movingAvg(arrNumbers, &sum, pos_avg, len, (int)rssi_beacon);*/


    // Select which laser range sensor readings to use
    if (multiranger_isinit) {
      front_range = (float)rangeGet(rangeFront) / 1000.0f;
      right_range = (float)rangeGet(rangeRight) / 1000.0f;
      left_range = (float)rangeGet(rangeLeft) / 1000.0f;
      back_range = (float)rangeGet(rangeBack) / 1000.0f;
      up_range = (float)rangeGet(rangeUp) / 1000.0f;
    }

    if (front_range > ranger_limit) {
      front_range = ranger_limit;
    }
    if (right_range > ranger_limit) {
      right_range = ranger_limit;
    }
    if (left_range > ranger_limit) {
      left_range = ranger_limit;
    }
    if(back_range > ranger_limit) {
      back_range = ranger_limit;
    }
    if (up_range > ranger_limit) {
      up_range = ranger_limit;
    } 


    // Get position estimate of kalman filter
    point_t pos;
    estimatorKalmanGetEstimatedPos(&pos);

    // Initialize setpoint
    memset(&setpoint_BG, 0, sizeof(setpoint_BG));

    // Filtere uprange, since it sometimes gives a low spike that
    up_range_filtered = update_median_filter_f(&medFilt, up_range);
    if (up_range_filtered < 0.05f) {
      up_range_filtered = up_range;
    }
    //up_range_filtered = 1.0f;
    //***************** Manual Startup procedure*************//

    //TODO: shut off engines when crazyflie is on it's back.

    /*    // indicate if top range is hit while it is not flying yet, then start counting
        if (keep_flying == false && manual_startup==false && up_range <0.2f && on_the_ground == true)
        {
          manual_startup = true;
          time_stamp_manual_startup_command = xTaskGetTickCount();
        }

        // While still on the ground, but indicated that manual startup is requested, keep checking the time
        if (keep_flying == false && manual_startup == true)
        {
            uint32_t currentTime = xTaskGetTickCount();
            // If 3 seconds has passed, start flying.
            if ((currentTime -time_stamp_manual_startup_command) > MANUAL_STARTUP_TIMEOUT)
            {
              keep_flying = true;
              manual_startup = false;
            }
        }*/

    // Don't fly if multiranger/updownlaser is not connected or the uprange is activated

    if (flowdeck_isinit && multiranger_isinit ) {
      //DEBUG_PRINT("correctly init both \n");
      correctly_initialized = true;
    }


#if METHOD == 3
    uint8_t rssi_beacon_threshold = 41;
       if (keep_flying == true && (!correctly_initialized || up_range < 0.2f || (!outbound
                                   && rssi_beacon_filtered < rssi_beacon_threshold))) {
         keep_flying = 0;
       }
#else
       if (keep_flying == true && (!correctly_initialized || up_range < 0.2f)) {
         keep_flying = 0;
       }
#endif

    state = 0;


/*

// Check RSSI of higher priority drones
    //DEBUG_PRINT("Checking RSSI\n");
    float rssi_inter_filtered = 140;
    float rssi_this_id;
    int i;
    for (i = 0; i < my_id; i++) {
      if (i % 2 != my_id % 2) {
        //DEBUG_PRINT("For drone id %i\n", i);
        rssi_this_id = get_median_filter_f(&medFiltDrones[i]);
        if (rssi_this_id < rssi_collision_threshold && rssi_this_id > 0) {
          rssi_inter_filtered = rssi_this_id;
          //DEBUG_PRINT("rssi_inter_filtered = %d\n", (int)rssi_inter_filtered);
          //DEBUG_PRINT("BREAK\n");
          break;
        }
      }
    }

    //DEBUG_PRINT("Passed in rssi = %d\n", (int)rssi_inter_filtered);

*/

    //DEBUG_PRINT("Timestamp %llu\n",usecTimestamp());


    // Main flying code
    if (keep_flying) {
      if (taken_off) {
        /*
         * If the flight is given a OK
         *  and the crazyflie has taken off
         *   then perform state machine
         */
    	  vel_w_cmd = 0;
        hover(&setpoint_BG, nominal_height);
        // DEBUG_PRINT("state_machine: Hover at nominal_height\n");

#if METHOD == 1 //WALL_FOLLOWING
        // wall following state machine
        state = wall_follower(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, right_range, heading_rad, 1);
#endif
#if METHOD ==2 //WALL_FOLLOWER_AND_AVOID
        // DEBUG_PRINT("state_machine: id_inter_closest = %i\n", id_inter_closest);
        // DEBUG_PRINT("state_machine: my_id = %i\n", my_id);
        // DEBUG_PRINT("state_machine: rssi array = %hhn\n", rssi_array_other_drones);
        // DEBUG_PRINT("state_machine: rssi of closest = %i\n", (int)rssi_angle_array_other_drones[id_inter_closest]);

        // // RSSI CA FOR 2 DRONES //
        // if (id_inter_closest > my_id || (id_inter_closest % 2 == my_id % 2)) {
        //     rssi_inter_filtered = 140;
            
        //     id_inter_closest = (uint8_t)find_minimum(rssi_array_other_drones, 40);
        // }

        rssi_inter_filtered = 140;
        state = wall_follower_and_avoid_controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range, right_range,
                heading_rad, rssi_inter_filtered);
#endif
#if METHOD==3 // SwWARM GRADIENT BUG ALGORITHM


/*
        bool priority = false;
        if (id_inter_closest > my_id) {
          priority = true;
        } else {
          priority = false;

        }
*/
bool priority = true;

        float drone_dist_from_wall;
        if (my_id % 2 == 1) {
          drone_dist_from_wall = drone_dist_from_wall_1;
        }
        else {
          drone_dist_from_wall = drone_dist_from_wall_2;
        }

        //TODO make outbound depended on battery.
        state = SGBA_controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf, front_range,
                                             left_range, right_range, back_range, heading_rad,
                                             (float)pos.x, (float)pos.y, rssi_beacon_filtered, rssi_inter_filtered, rssi_angle_inter_closest, priority, outbound, drone_dist_from_wall);

        memcpy(&p_reply.data[1],&rssi_angle, sizeof(float));



#endif

        // convert yaw rate commands to degrees
        float vel_w_cmd_convert = vel_w_cmd * 180.0f / (float)M_PI;

        // Convert relative commands to world commands (not necessary anymore)
        /*float psi = heading_rad;
        float vel_x_cmd_convert =  cosf(-psi) * vel_x_cmd + sinf(-psi) * vel_y_cmd;
        float vel_y_cmd_convert = -sinf(-psi) * vel_x_cmd + cosf(-psi) * vel_y_cmd;*/
        //float vel_y_cmd_convert = -1 * vel_y_cmd;
        // DEBUG_PRINT("state_machine: Calling vel_command\n");
        vel_command(&setpoint_BG, vel_x_cmd, vel_y_cmd, vel_w_cmd_convert, nominal_height);
        on_the_ground = false;
      } else {
        /*
         * If the flight is given a OK
         *  but the crazyflie  has not taken off
         *   then take off
         */
          // if (usecTimestamp() >= takeoffdelaytime + 1000*1000*my_id) {
          if (usecTimestamp() >= takeoffdelaytime) {

              take_off(&setpoint_BG, nominal_height);
              if (height > nominal_height) {
                  taken_off = true;


#if METHOD==1 // wall following
          wall_follower_init(drone_dist_from_wall, drone_speed, 1);
#endif
#if METHOD==2 // wallfollowing with avoid
          if (my_id%2==1)
          init_wall_follower_and_avoid_controller(drone_dist_from_wall_1, drone_speed, -1);
          else
          init_wall_follower_and_avoid_controller(drone_dist_from_wall_2, drone_speed, 1);

#endif
#if METHOD==3 // Swarm Gradient Bug Algorithm

          // float angle_interval = (180.0f / (number_of_angles-1));

          uint8_t my_id_dec = my_id;
          if (my_id > 9) {
            my_id_dec = my_id - 6;
          } else if (my_id > 19) {
            my_id_dec = my_id - 12;
          } 
          DEBUG_PRINT("id = %i\n", my_id_dec);

          // Testing
          // float heading = -90.0f + angle_interval * (my_id_dec % number_of_angles);
          // // float heading_180;
          // if (heading >= 0) {
          //   heading = (heading / 180 - (int)(heading / 180)) * 180;
          // } else {
          //   heading = (heading / 180 + (int)(heading / 180)) * 180;
          // }

          // 8 and 9 directions
          // DEBUG_PRINT("heading = %.2f\n", (double)heading);
          // if (my_id_dec % 2 == 1) {
          //   init_SGBA_controller(drone_dist_from_wall_1, drone_speed, heading, -1);
          // } else {
          //   init_SGBA_controller(drone_dist_from_wall_2, drone_speed, heading, 1);
          // }

          // static float heading = 0.0f;
          // static float heading[3] = { 0.0f, 0.0f, 0.0f };
          static float heading[3] = { 60.0f, 0.0f, -60.0f };
          // static float heading[15] = { -69.0f, -48.0f, -27.0f, -6.0f, 15.0f, 36.0f, 57.0f, 78.0f, -66.0f, -42.0f, -18.0f, 6.0f, 30.0f, 54.0f, 78.0f};
          // int index = robot_id % 3;
          // DEBUG_PRINT("heading = %.2f\n", (double)heading);
          if (my_id_dec % 2 == 1) {
            init_SGBA_controller(drone_dist_from_wall_1, drone_speed, heading[my_id-1], -1);
          } else {
            init_SGBA_controller(drone_dist_from_wall_2, drone_speed, heading[my_id-1], 1);
          }
          

#endif


              }
          on_the_ground = false;
          }else{
              shut_off_engines(&setpoint_BG);
              taken_off = false;
          }

      }
    } else { //keep_flying == false
      if (taken_off) {
        /*
         * If the flight is given a not OK
         *  but the crazyflie  has already taken off
         *   then land
         */
        if (move_away_from_walls) {
        DEBUG_PRINT("MOVE AWAY FROM WALLS AND LAND \n");
        
          const float VELOCITY = 0.1; 
          vel_x_cmd = 0; vel_y_cmd = 0;
          const float threshold = 1.1;
          const float buffer1 = 0.2;
          const float buffer2 = 0.15;


          // if (front_range < threshold + buffer1) {
          //   DEBUG_PRINT("FRONT %.2f < THRESHOLD %.2f \n", (double)front_range, (double)(threshold + buffer1));
          //   if (front_range < threshold) {
          //     vel_x_cmd -= VELOCITY;
          //   } else {
          //     vel_x_cmd += VELOCITY;
          //   }
          //   if (front_range > threshold && front_range < threshold + buffer2){
          //     move_away_from_walls = false;
          //     DEBUG_PRINT("Finish Moving Away\n");
          //   }
          // }
          if (left_range + right_range < 2*threshold + buffer1) {
            DEBUG_PRINT("LEFT AND RIGHT < THRESHOLD \n");
            if (left_range < threshold) {
              vel_y_cmd -= VELOCITY;
            } else {
              vel_y_cmd += VELOCITY;
            }
            if (left_range > threshold && left_range < threshold + buffer2){
              move_away_from_walls = false;
              DEBUG_PRINT("Finish Moving Away\n");
            }
          }



          // check multiranger distance
          else if (is_close(front_range) || is_close(left_range) || is_close(right_range)) { //|| is_close(back_range) 
            DEBUG_PRINT("------ \n");
           
            if (is_close(front_range)) {
                DEBUG_PRINT("Move Backwards\n");
                vel_x_cmd -= VELOCITY;
            }
            // if (is_close(back_range)) {
            //     DEBUG_PRINT("Move Forward\n");
            //     vel_x_cmd += VELOCITY;
            // }
            if (is_close(left_range)) {
                DEBUG_PRINT("Move Right\n");
                vel_y_cmd -= VELOCITY;
            }
            if (is_close(right_range)) {
                DEBUG_PRINT("Move Left\n");
                vel_y_cmd += VELOCITY;
            }
          }

          else{
            move_away_from_walls = false;
            DEBUG_PRINT("Finish moving away\n");
          }
          
          float vel_w_cmd_convert = vel_w_cmd * 180.0f / (float)M_PI;
          vel_command(&setpoint_BG, vel_x_cmd, vel_y_cmd, vel_w_cmd_convert, nominal_height);
        }
  
         

      
        else //original land logic
        {
          DEBUG_PRINT("LANDING\n");
          vTaskDelay(1800));
          land(&setpoint_BG, 0.75f);
          if (height < 0.1f) {
            shut_off_engines(&setpoint_BG);
            taken_off = false;
          }
          on_the_ground = false;
        } //else original land topic
      


      } else {


        /*
         * If the flight is given a not OK
         *  and crazyflie has landed
         *   then keep engines off
         */
        shut_off_engines(&setpoint_BG);
        takeoffdelaytime=usecTimestamp();
        on_the_ground = true;
      }
    }

#if METHOD != 1
    if (height > 0.25f && up_range > 0.2f) {
      // DEBUG_PRINT("height: %.2f\n", (double)height);
      // DEBUG_PRINT("up range: %.2f\n", (double)up_range);
      is_flying = true;
    }
    else {
      // DEBUG_PRINT("NOT FLYING\n");
      is_flying = false;
    }

    if ((usecTimestamp() >= radioSendBroadcastTime + 1000*500) && (is_flying == true)) {
        radiolinkSendP2PPacketBroadcast(&p_reply);
        radioSendBroadcastTime = usecTimestamp();
        // DEBUG_PRINT("state_machine: Broadcasting RSSI\n");
    }

#endif
    commanderSetSetpoint(&setpoint_BG, STATE_MACHINE_COMMANDER_PRI);

  }
}

void p2pcallbackHandler(P2PPacket *p)
{
    id_inter_ext = p->data[0];
    //DEBUG_PRINT("receive packet \n");


    if (id_inter_ext == 0x63)
    {
        // get the drone's ID
        uint64_t address = configblockGetRadioAddress(); 
        uint8_t my_id =(uint8_t)((address) & 0x00000000ff); 

        //if 3rd byte of packet = 0xff or = drone's ID
        if (p->data[2] == 0xff || p->data[2] == my_id) 
        {
         if (p->data[1] == 0 || p->data[1] == 1)
         {
          keep_flying =  p->data[1];
         }
         else if (p->data[1] == 2)
         {
          keep_flying = false;
          move_away_from_walls = true;
         }
        }

        // if (p->data[2] == my_id){
        //   keep_flying = !keep_flying; 
        // }
        // else{
        //   keep_flying =  p->data[1];
        // }


    }else if(id_inter_ext == 0x64){ 
        rssi_beacon =p->rssi;

    }
    else{
        rssi_inter = p->rssi;
        DEBUG_PRINT("state_machine: Received RSSI is %i\n", rssi_inter);
        memcpy(&rssi_angle_inter_ext, &p->data[1], sizeof(float));

        rssi_array_other_drones[id_inter_ext] = rssi_inter;
        time_array_other_drones[id_inter_ext] = usecTimestamp();
        rssi_angle_array_other_drones[id_inter_ext] = rssi_angle_inter_ext;

        // //Update filter for drones
        update_median_filter_f(&medFiltDrones[id_inter_ext], (float)rssi_inter);




      //   // FOR DEBUGGING //

      //   // Print medFiltDrones
      //   int result = update_median_filter_f(&medFiltDrones[id_inter_ext], (float)rssi_inter);
      //   DEBUG_PRINT("For drone %i\n", id_inter_ext);
      //   for (int i = 0; i < medFiltDrones[id_inter_ext].size; i++)
      //   {
      //     float temp = medFiltDrones[id_inter_ext].data[i];
      //     DEBUG_PRINT("%i ", (int)temp);
      //   }
      //   DEBUG_PRINT("state_machine: Median result is %i\n", (int)result);

      //   // Print RSSI checking
      //   DEBUG_PRINT("Checking RSSI\n");
      //   float rssi_inter_filtered = 140;
      //   float rssi_this_id;
      //   int i;
      //   uint64_t address = configblockGetRadioAddress();
      //   uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
      //   for (i = 0; i < my_id; i++) {
      //     DEBUG_PRINT("For drone %i\n", i);
      //     if (i % 2 != my_id % 2) {
      //       rssi_this_id = get_median_filter_f(&medFiltDrones[i]);
      //       DEBUG_PRINT("rssi_this_id = %d\n", (int)rssi_this_id);
      //       if (rssi_this_id < rssi_collision_threshold && rssi_this_id > 0) {
      //         rssi_inter_filtered = rssi_this_id;
      //         DEBUG_PRINT("BREAK\n");
      //         break;
      //       }
      //     }
      //   }
      // DEBUG_PRINT("rssi_inter_filtered = %d\n", (int)rssi_inter_filtered);



    }



}

PARAM_GROUP_START(statemach)
PARAM_ADD(PARAM_UINT8, keep_flying, &keep_flying)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, corinit, &correctly_initialized)
PARAM_GROUP_STOP(statemach)

LOG_GROUP_START(statemach)
LOG_ADD(LOG_UINT8, state, &state)
LOG_ADD(LOG_UINT8, rssi_inter, &rssi_beacon)
LOG_ADD(LOG_UINT8, rssi_beacon, &rssi_beacon_filtered)
LOG_GROUP_STOP(statemach)
