#include "particle_swarm.h"

// Particle swarm optimisation algorithm
// ref: https://arxiv.org/pdf/2008.10000

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "stabilizer_types.h"

Particle swarm[SWARM_SIZE];

float w = 0.5; // Inertia coefficient
float c1 = 1.5; // Cognitive coefficient
float c2 = 1.5; // Social coefficient
float scaling_factor = 1.0;

static float global_best_position_x = 0.0;
static float global_best_position_y = 0.0;
static float global_best_position_z = 0.0;
static float global_best_value = FLT_MAX;

static float safe_distance_from_wall = 0.0f;
// TODO eventually
// find the pattern for the inttial pos of drone(s)

void init_particle_swarm_controller(float velocity_x, float velocity_y, float velocity_z, float drone_position_x, float drone_position_y, float drone_position_z, float personal_best_value) {
  for (int i = 0; i < SWARM_SIZE; i++) {
    
    // Initialize position
    swarm[i].position_x = drone_position_x;
    swarm[i].position_y = drone_position_y;
    swarm[i].position_z = drone_position_z;

    // Initialize velocity
    swarm[i].velocity_x = velocity_x * rand_float();
    swarm[i].velocity_y = velocity_y * rand_float();
    swarm[i].velocity_z = velocity_z * rand_float();


    // Initialize personal best position
    swarm[i].personal_best_position_x = drone_position_x;
    swarm[i].personal_best_position_y = drone_position_y;
    swarm[i].personal_best_position_z = drone_position_z;

    // Evaluate initial fitness
    swarm[i].personal_best_value = evaluate_fitness(&swarm[i]);

  }
}

int particle_swarm_controller(setpoint_t *sp) {

    for (int i = 0; i < SWARM_SIZE; i++) {
        // Update particle velocity
        swarm[i].velocity_x = w * swarm[i].velocity_x +
                              c1 * rand_float() * (swarm[i].personal_best_position_x - swarm[i].position_x) +
                              c2 * rand_float() * (global_best_position_x - swarm[i].position_x);

        swarm[i].velocity_y = w * swarm[i].velocity_y +
                              c1 * rand_float() * (swarm[i].personal_best_position_y - swarm[i].position_y) +
                              c2 * rand_float() * (global_best_position_y - swarm[i].position_y);

        swarm[i].velocity_z = w * swarm[i].velocity_z +
                              c1 * rand_float() * (swarm[i].personal_best_position_z - swarm[i].position_z) +
                              c2 * rand_float() * (global_best_position_z - swarm[i].position_z);

        // Update particle position
        swarm[i].position_x += swarm[i].velocity_x;
        swarm[i].position_y += swarm[i].velocity_y;
        swarm[i].position_z += swarm[i].velocity_z;

        // Evaluate fitness
        float fitness_value = evaluate_fitness(&swarm[i]);

        // Update personal best if needed
        if (fitness_value < swarm[i].personal_best_value) {
            swarm[i].personal_best_value = fitness_value;
            swarm[i].personal_best_position_x = swarm[i].position_x;
            swarm[i].personal_best_position_y = swarm[i].position_y;
            swarm[i].personal_best_position_z = swarm[i].position_z;
        }

        // Update global best if needed
        if (fitness_value < global_best_value) {
            global_best_value = fitness_value;
            global_best_position_x = swarm[i].position_x;
            global_best_position_y = swarm[i].position_y;
            global_best_position_z = swarm[i].position_z;
        }
    }

    // Set the drone's velocity based on the global best position
    sp->velocity.x = (global_best_position_x - sp->position.x) * scaling_factor;
    sp->velocity.y = (
       - sp->position.y) * scaling_factor;
    sp->velocity.z = (global_best_position_z - sp->position.z) * scaling_factor;

    return 0;
}

// Static helper functions
static float rand_float() {
    return (float)rand() / (float)(RAND_MAX);
}

static float evaluate_fitness(Particle *p) {

    // The fitness is the distance from the wall
    float distance = sqrt(pow(p->position_x - safe_distance_from_wall, 2) +
                          pow(p->position_y - safe_distance_from_wall, 2) +
                          pow(p->position_z - safe_distance_from_wall, 2));
    return distance; 
}

// function for drone to hover
static void commandHover(float *vel_x, float *vel_y, float *vel_w)
{
  *vel_x = 0.0;
  *vel_y = 0.0;
  *vel_w = 0.0;
}

