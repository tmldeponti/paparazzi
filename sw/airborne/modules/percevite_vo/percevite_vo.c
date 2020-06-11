/*
 * Copyright (C) nilay994 <nilay994>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.

 */
/** @file "modules/percevite_vo/percevite_vo.h"
 * @author nilay994 <nilay994>
 * Percevite's Velocity Obsctacle for GPS and avoid
 * don't make AVOID waypoints go too near the real flightplan waypoints
 */

 /* printf and fprintf */
#include <stdio.h>

/* MAKE MATRIX PTR */
#include "math/pprz_algebra_float.h"

/* Binary Floating Point for geodetic */
#include "math/pprz_algebra_int.h"

#include "modules/percevite_vo/percevite_vo.h"

/* for flightplan edit */
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"

/* gps functions */
#include "state.h"

volatile bool percevite_requires_avoidance = false;

/* externed from percevite_wifi */
drone_data_t dr_data[MAX_DRONES];

void polar2cart(float mag, float directn, float *cart) {
  cart[0] = mag * cosf(directn);
  cart[1] = mag * sinf(directn);
}

void cart2polar(float *vel_vec, float *mag, float *dir) {
  *mag = float_vect_norm(vel_vec, 2);
  *dir = atan2f(vel_vec[1], vel_vec[0]);
}

void calc_proj_matrix(const float *a, const float *b, float **c) {
  float inner_product = float_vect_dot_product(a, b, 2);
  if (fabsf(inner_product) < 0.001) {
    inner_product = 0.001;
  }
  c[0][0] = a[0] * b[0] / inner_product;
  c[0][1] = a[0] * b[1] / inner_product;
  c[1][0] = a[1] * b[0] / inner_product;
  c[1][1] = a[1] * b[1] / inner_product;
}

void percevite_vo_resolve_by_project(const drone_data_t *robot_a, float angle1, float angle2, float *centre, float *new_vel_a_bdy) {
    
    float vel_a_w[2];

    // body frame to world frame velocity vector
    vel_a_w[0] = robot_a->pos.x + robot_a->vel.x;
    vel_a_w[1] = robot_a->pos.y + robot_a->vel.y;

    float yintercept = centre[1] - (tanf(angle1) * centre[0]);
    float xintercept = 0.0;

    // if slope is zero, projection is the x-coordinate itself
    if (angle2 == 0) {
      new_vel_a_bdy[0] = (vel_a_w[0] - robot_a->pos.x);
      new_vel_a_bdy[1] = -robot_a->pos.y;
      return;
    } else {
      xintercept = - yintercept / tanf(angle1);
    }

    float line_prop[3] = {xintercept, yintercept, tanf(angle1)};

    // slope-intercept form, poly1d(m, c), line = poly1d([line_prop[2], line_prop[1]])
    // calculate vector passing through origin, y = mx
    float vector_from_line[2] = {line_prop[0], -line_prop[1]};

    // prevent divide by zero
    if (float_vect_dot_product(vector_from_line, vector_from_line, 2) > 0.1) {
        // calculate projection matrix with property P**2 = P
        float _projection_matrix[2][2] = {0};
        MAKE_MATRIX_PTR(projection_matrix, _projection_matrix, 2);

        calc_proj_matrix(vector_from_line, vector_from_line, projection_matrix);

        float cc_vector[2] = {vel_a_w[0], (vel_a_w[1] - line_prop[1])};
        float projected_pt[2];
        float_mat_vect_mul(projected_pt, projection_matrix, cc_vector, 2, 2);
        projected_pt[1] = projected_pt[1] + line_prop[1];

        // convert back to body frame velocities, make this the new body frame velocity
        new_vel_a_bdy[0] = projected_pt[0] - robot_a->pos.x;
        new_vel_a_bdy[1] = projected_pt[1] - robot_a->pos.y;

        #ifdef DBG_VEL_OBS
          printf("resolved: %f \t %f (m/s)\n", new_vel_a_bdy[0], new_vel_a_bdy[1]);
        #endif
        
    } else {
        // if norm is lesser, give back old vel.. (not sure)
        new_vel_a_bdy[0] = robot_a->vel.x;
        new_vel_a_bdy[1] = robot_a->vel.y;
    }
    // return resolved velocity in cartesian;
}

void percevite_vo_detect(const drone_data_t *robot1, const drone_data_t *robot2) {
  
  float drel[2], vrel[2]; 
  drel[0] = robot1->pos.x - robot2->pos.x;
  drel[1] = robot1->pos.y - robot2->pos.y;

  vrel[0] = robot1->vel.x - robot2->vel.x;
  vrel[1] = robot1->vel.y - robot2->vel.y;

  float norm_vrel = float_vect_norm(vrel, 2);
  float norm_drel = float_vect_norm(drel, 2);

  static uint16_t positive_ctr = 0;
  static uint16_t negative_ctr = 0;

  // both are barely moving and far apart
  if ((norm_vrel < GPS_ACCURACY_VEL) && (norm_drel > 3 * RR)) {
    negative_ctr += 1;
    return;
  }

  // details on these two variables in Dennis Wijngaarden's prelim
  float tcpa = -float_vect_dot_product(drel, vrel, 2) / pow(norm_vrel, 2);
  float dcpa = sqrtf((fabsf(pow(norm_drel, 2) - (pow(tcpa, 2) * pow(norm_vrel, 2)))));
  // printf("tcpa: %f, dcpa: %f\n", tcpa, dcpa);

  float angleb = atan2f((robot2->pos.y - robot1->pos.y), (robot2->pos.x - robot1->pos.x));
  float deltad = norm_drel;
  float angleb1 = angleb - atanf(RR / deltad);
  float angleb2 = angleb + atanf(RR / deltad);

  float centre[2];
  centre[0] = robot1->pos.x + robot2->vel.x; 
  centre[1] = robot1->pos.y + robot2->vel.y;

  static uint16_t high = 0;
  static uint16_t low = 0;

  // check if you must pick a smart non default direction of avoidance
  // default (lower angle w.r.t. to North axis)
  static bool non_default_dir = 0;

  #ifdef DBG_VEL_OBS
    printf("pos: %d, neg: %d \n", positive_ctr, negative_ctr);
  #endif
  // collision is imminent if tcpa > 0 and dcpa < RR
  if ((tcpa > 0) && (tcpa < ETA_AVOID) && (dcpa < RR)) {
    // positive re-inforcement towards final decision
    positive_ctr += 1;
    negative_ctr = 0;
    
    float obs_vel_norm = sqrtf(robot2->vel.y * robot2->vel.y + robot2->vel.x * robot2->vel.x);
    
    // chose direction of evasion before positive reinforcement is locked
    if (obs_vel_norm > 1.5 && positive_ctr < 20) {
      // high speed obs avoidance: activate non-default dir
      non_default_dir = 1;

      float obs_phase = atan2f(robot2->vel.y, robot2->vel.x);
      printf("obs: %f, b1: %f, b2: %f\n", R2D * obs_phase, R2D * angleb1, R2D * angleb2);

      float unit_v_0[2] = {cosf(obs_phase), sinf(obs_phase)};
      float unit_v_1[2] = {cosf(angleb1), sinf(angleb1)};
      float unit_v_2[2] = {cosf(angleb2), sinf(angleb2)};

      float v0v1 = float_vect_dot_product(unit_v_0, unit_v_1, 2);
      float v0v2 = float_vect_dot_product(unit_v_0, unit_v_2, 2);

      // printf("v0v1: %f, v0v2: %f\n", v0v1, v0v2);
      /* pick the side of velobs which has a bigger phase difference than obstacle's phase use a 
         fuzzy, decision counter - avoids confusion in co-ordination and GPS measurement noise */
      
      // dot product is lower for vector which is least aligned with obstacle
      if(v0v1 <= v0v2) {
        low += 1;
      } else {
        high += 1;
      }
    }
  }
  #ifdef DBG_VEL_OBS
    printf("low: %d, high: %d\n", low, high);
  #endif
  
  else {
    if (tcpa < -0.5) {
      // if no collisions are predicted let flightplan take over..
      negative_ctr += 1;
      positive_ctr = 0;
    }
  }
  
  // 20 at 10 Hz = 2 seconds
  if (positive_ctr > 20) {
    float newvel_cart[2];
    // choose left or right
    if(non_default_dir) {
      if(low > high) {
        percevite_vo_resolve_by_project(robot1, angleb1, angleb2, centre, newvel_cart);
        #ifdef DBG_VEL_OBS
          printf("...........LOWER ANGLE\n");
        #endif
      } else {
        percevite_vo_resolve_by_project(robot1, angleb2, angleb1, centre, newvel_cart);
        #ifdef DBG_VEL_OBS
          printf("...........HIGHER ANGLE\n");
        #endif
      }
    } else {
      percevite_vo_resolve_by_project(robot1, angleb1, angleb2, centre, newvel_cart);
      #ifdef DBG_VEL_OBS
        printf("...........LOWER ANGLE\n");
      #endif
    }

    // initializing resolution vel @ 0.0 = stop at current waypoint
    static float resolution_cmd[2] = {0.0, 0.0};
    if (deltad > RR) {
      resolution_cmd[0] = min(max(newvel_cart[0], -MAX_VEL), MAX_VEL);
      resolution_cmd[1] = min(max(newvel_cart[1], -MAX_VEL), MAX_VEL);
    } // else continue last_resolution_cmd
    struct EnuCoor_i new_coord;
    // todo verify x y ned enu swap
    new_coord.x = POS_BFP_OF_REAL(robot1->pos.x + VEL_STEP * resolution_cmd[0]); // TODO: vel*dt
    new_coord.y = POS_BFP_OF_REAL(robot1->pos.y + VEL_STEP * resolution_cmd[1]); // TODO: vel*dt
    
    // lock flightplan to percevite_vo module's AVOID waypoint
    waypoint_move_xy_i(WP_AVOID, new_coord.x, new_coord.y);
    printf("[OBS] mode\n"); 
    //new co-ord: %f, %f\n", waypoint_get_x(WP_AVOID), waypoint_get_y(WP_AVOID));
    percevite_requires_avoidance = true;
  }

  if (negative_ctr > 20) {
    // Avoided! Handoff to flightplan 
    printf("[POS] mode\n");
    percevite_requires_avoidance = false;
  }

  if (negative_ctr > 50) {
    // clear L or R only well after avoidance
    high = 0;
    low = 0;
    non_default_dir = 0;
  }
}

void vo_simulate_loop(drone_data_t* robot_sim) {
  robot_sim->pos.x += robot_sim->vel.x;
  robot_sim->pos.y += robot_sim->vel.y;
}

drone_data_t drone1, drone2, obstacle_dr;
void percevite_vo_init(void) {
  percevite_requires_avoidance = false;
  printf("VO init done\n");

  /*********** if mode == SIM ****************/
  #ifdef SIMMODE
  drone2.pos.x = -10.0;
  drone2.pos.y = -10.0;
  drone2.vel.x = 0.12;
  drone2.vel.y = 0.12;
  #endif
}

void percevite_vo_simulate_loop(void) {
  #ifdef SIMMODE
    vo_simulate_loop(&drone2);
  #endif
}


/* 5Hz periodic loop */
void percevite_vo_periodic(void) {

  // simulate what happens to both robots to later extern it to Percevite WiFi
  // overwrite dr_data for tx...  
  drone1 = dr_data[1];

  // if VEL_OBS is in SIM mode, then don't listen to WiFi module for obstacle co-ordinates
  #ifndef SIMMODE
    drone2 = dr_data[2];
  #endif

  #if SELF_ID == 1
    obstacle_dr = drone2;
  #else 
    obstacle_dr = drone1;
  #endif

  /*********** else if mode == AP ****************/
  struct EnuCoor_i obstacle;
  obstacle.x = POS_BFP_OF_REAL(obstacle_dr.pos.x);
  obstacle.y = POS_BFP_OF_REAL(obstacle_dr.pos.y);
  waypoint_move_xy_i(WP_OBS, obstacle.x, obstacle.y);

  // get altitude from rest of the waypoints
  float alt_from_p1 = waypoint_get_alt(WP_p1);
  waypoint_set_alt(WP_AVOID, alt_from_p1);

  #ifdef DBG_VEL_OBS
    printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
          drone1.pos.x, drone1.pos.y, drone1.vel.x, drone1.vel.y,
          drone2.pos.x, drone2.pos.y, drone2.vel.x, drone2.vel.y);
  #endif
  
  percevite_vo_detect(&drone1, &drone2);

}