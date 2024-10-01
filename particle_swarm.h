#ifndef PARTICLE_SWARM_H
#define PARTICLE_SWARM_H

#include <float.h>  // For INFINITY
#include <stdlib.h> // For rand()


#define SWARM_SIZE 1

// Define the structure for a particle
typedef struct {
    float position_x;
    float position_y;
    float position_z;
    float velocity_y;
    float velocity_x;
    float velocity_z;            
    float personal_best_position_x;
    float personal_best_position_y;
    float personal_best_position_z;  
    float personal_best_value;      
} Particle;



// Swarm of particles
extern Particle swarm[SWARM_SIZE];

// Global best position and fitness (fitness starts at infinity)
extern float global_best_position_x;
extern float global_best_position_y;
extern float global_best_position_z;
extern float global_best_fitness;
// Maximum velocity allowed per dimension
extern float max_velocity_x;
extern float max_velocity_y;
extern float max_velocity_z;

// Reference distance (used in fitness calculation or behavior adjustments)
extern float ref_distance_from_wall;

// Function declarations
void init_particle_swarm_controller(float velocity_x, float velocity_y, float velocity_z, float drone_position_x, float drone_position_y, float drone_position_z, float personal_best_value);

int particle_swarm_controller(setpoint_t *sp);

#endif // PARTICLE_SWARM_H
