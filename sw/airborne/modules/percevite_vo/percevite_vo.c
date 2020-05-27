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

    // increase predicted vel.. measurement noise, feedfwd for drag
    vel_a_w[0] += 0.5;
    vel_a_w[1] += 0.5;

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

        printf("resolved: %f \t %f (m/s)\n", new_vel_a_bdy[0], new_vel_a_bdy[1]);
        
    } else {
        // if norm is lesser, give back old vel.. (not sure)
        new_vel_a_bdy[0] = robot_a->vel.x;
        new_vel_a_bdy[1] = robot_a->vel.y;
    }
    // return resolved velocity in cartesian;
}

bool percevite_vo_detect(const drone_data_t *robot1, const drone_data_t *robot2, float *resolution_cmd) {
  
  float drel[2], vrel[2]; 
  drel[0] = robot1->pos.x - robot2->pos.x;
  drel[1] = robot1->pos.y - robot2->pos.y;

  vrel[0] = robot1->vel.x - robot2->vel.x;
  vrel[1] = robot1->vel.y - robot2->vel.y;

  float norm_vrel = float_vect_norm(vrel, 2);
  if (norm_vrel < 0.1) {
    norm_vrel = 0.1;
  }

  // details on these two variables in Dennis Wijngaarden's prelim
  float tcpa = -float_vect_dot_product(drel, vrel, 2) / pow(norm_vrel, 2);
  float dcpa = sqrtf((fabsf(pow(float_vect_norm(drel, 2), 2) - (pow(tcpa, 2) * pow(float_vect_norm(vrel, 2), 2)))));
  // printf("tcpa: %f, dcpa: %f\n", tcpa, dcpa);

  float angleb = atan2f((robot2->pos.y - robot1->pos.y), (robot2->pos.x - robot1->pos.x));
  float deltad = float_vect_norm(drel, 2);
  float angleb1 = angleb - atan(RR / deltad);
  float angleb2 = angleb + atan(RR / deltad);

  float centre[2];
  centre[0] = robot1->pos.x + robot2->vel.x; 
  centre[1] = robot1->pos.y + robot2->vel.y;

  // avoid calling the avoidance funtion after reaching the waypoint. 
  // "percevite_requires_avoidance" will only be true if the destination is far
  float closest_wp_dist = 100; // 100 meters
  for (int i = 0; i < WP_p1 + 5; i++) {
    float current_distance = get_dist2_to_waypoint(i);
    if (current_distance < closest_wp_dist) {
      closest_wp_dist = current_distance;
    }
  }
  
  // collision is imminent if tcpa > 0 and dcpa < RR && (deltad > RR)... creating trouble..
  if ((tcpa > 0) && (dcpa < RR) && (deltad > RR) && (closest_wp_dist > RR)) {
    // TODO: non mirroring conditions to avoid!! p.s.: heading is lost now if (va[1] < vrel[1]) ...
    float newvel_cart[2];
    percevite_vo_resolve_by_project(robot1, angleb1, angleb2, centre, newvel_cart);

    /* TODO: rate and mag bound here and send to ctrl outerloop */
    resolution_cmd[0] = min(max(newvel_cart[0], -MAX_VEL), MAX_VEL);
    resolution_cmd[1] = min(max(newvel_cart[1], -MAX_VEL), MAX_VEL);
    
    return true;
  }
  // if no collisions are predicted, don't resolve and be what you were
  // send the original velocity command back..
  else {
    resolution_cmd[0] = 0.5;  // 0.5 m/s
    resolution_cmd[1] = 0;
    return false;
  }
}

void vo_simulate_loop(drone_data_t* robot_sim) {
  robot_sim->pos.x += robot_sim->vel.x;
  robot_sim->pos.y += robot_sim->vel.y;
}

drone_data_t drone1, drone2;
void percevite_vo_init(void) {
  percevite_requires_avoidance = false;
  printf("VO init done\n");

  // VO init not required when not SIMulating
}


/* 5Hz periodic loop */
void percevite_vo_periodic(void) {

  // simulate what happens to both robots to later extern it to Percevite WiFi
  // overwrite dr_data for tx...  
  drone1 = dr_data[1];

  drone2 = dr_data[2];
  // sitting duck (sim)
  /*********** if mode == NPS ****************/
  // drone2.pos.x = waypoint_get_x(WP_DRONE2);
  // drone2.pos.y = waypoint_get_y(WP_DRONE2);
  // drone2.vel.x = 0.0;
  // drone2.vel.y = 0.0;
  /*********** else if mode == AP ****************/
  struct EnuCoor_i drone2cor;
  drone2cor.x = POS_BFP_OF_REAL(drone2.pos.x);
  drone2cor.y = POS_BFP_OF_REAL(drone2.pos.y);
  waypoint_move_xy_i(WP_DRONE2, drone2cor.x, drone2cor.y);

  printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
        drone1.pos.x, drone1.pos.y, drone1.vel.x, drone1.vel.y,
        drone2.pos.x, drone2.pos.y, drone2.vel.x, drone2.vel.y);
  
  // resolution command changes the vel and head of robot1 to avoid collision
  float resolution_cmd[2];
  percevite_requires_avoidance = percevite_vo_detect(&drone1, &drone2, resolution_cmd);

  if (percevite_requires_avoidance) {
    struct EnuCoor_i new_coord;
    // todo verify x y ned enu swap
    new_coord.x = POS_BFP_OF_REAL(drone1.pos.x + 0.5 * RR * resolution_cmd[0]); // TODO: vel*dt
    new_coord.y = POS_BFP_OF_REAL(drone1.pos.y + 0.5 * RR * resolution_cmd[1]); // TODO: vel*dt
    waypoint_move_xy_i(WP_AVOID, new_coord.x, new_coord.y);
    printf("[OBS] mode\n"); //new co-ord: %f, %f\n", waypoint_get_x(WP_AVOID), waypoint_get_y(WP_AVOID));
  } else {
    printf("[POS] mode\n");
  }
}