// drone_variables.h

#ifndef DRONE_VARIABLES_H
#define DRONE_VARIABLES_H

extern float drone_dist_from_wall_1;
extern float drone_dist_from_wall_2;
extern float drone_speed;
extern float drone_heading_threshold;

extern float drone_dist_from_wall_corner_margin;
extern int drone_speed_corner_scale;
extern float drone_dist_from_wall_forward_margin;
extern int drone_speed_forward_adjust_scale;
extern float drone_dist_from_wall_to_start_margin;

extern int rssi_collision_threshold;
extern float rssi_reset_interval;
extern float nominal_height;
// extern float CA_height;
extern int number_of_angles;

#endif // DRONE_VARIABLES_H