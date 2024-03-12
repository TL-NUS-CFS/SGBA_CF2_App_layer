// drone_variables.c

#include "drone_variables.h"

// WALL FOLLOWING: DRONE DIST FROM WALL VARIABLES
float drone_dist_from_wall_1 = 0.6; // following on the left side of the wall
float drone_dist_from_wall_2 = 1.2; // following on the right side of the wall
float drone_dist_from_wall_corner_margin = 0.3; // error margin for turning around corner
float drone_dist_from_wall_forward_margin = 0.1; // error margin for going forward along wall
float drone_dist_from_wall_to_start_margin = 0.2; // error margin for transitions (forward/corner)
float drone_heading_threshold = 0.8; // in rad / Check if drone heading goes over 0.8 rad

// WALL FOLLOWING: DRONE SPEED VARIABLES
float drone_speed = 0.2; // default drone speed
int drone_speed_corner_scale = 3; // scale down for turning around corner
int drone_speed_forward_adjust_scale = 2; // scale down for going forward along wall

// RSSI COLLISION AVOIDANCE VARIABLES
int rssi_collision_threshold = 60;
float rssi_reset_interval = 3.0;

float nominal_height = 0.3;
// float CA_height = 1.3;

// SGBA VARIABLES
// int number_of_angles = 5; // number of preferred directions for drone to follow
