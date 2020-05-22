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

#include "modules/percevite_vo/percevite_vo.h"

/* gps functions */
#include "state.h"

volatile bool collision = false;

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
    // polar2cart(robot_a->vel, robot_a->head, vel_a_bdy);

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

        // new_vel_a_bdy[0] = new_vel_a_bdy[0] + 0.4;

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

  // collision is imminent if tcpa > 0 and dcpa < RR
  // && (deltad > RR)... creating trouble..
  if ((tcpa > 0) && (dcpa < RR)) {
    // TODO: conditions to avoid!! Heading is lost now.. 
    //if (va[1] < vrel[1]) ...
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
    resolution_cmd[0] = 0.5;
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
  // VO init not required when not SIMulating
}

/* 5Hz periodic loop */
void percevite_vo_periodic(void) {

  // simulate what happens to both robots to later extern it to Percevite WiFi
  // overwrite dr_data for tx...  
  drone1 = dr_data[1];
  drone2 = dr_data[2];

  // resolution command changes the vel and head of robot1 to avoid collision
  float resolution_cmd[2];
  
  if (percevite_vo_detect(&drone1, &drone2, resolution_cmd)) {
    printf("[VELOBS] mode\n");
    collision = true;
    // resolve yaw? Cartesian mode: translate in lateral plane rather than change heading..
    lateral_vel_ctrl(resolution_cmd[0] + 0.2, resolution_cmd[1]);
  } else {
    printf("[POS] mode\n");
    collision = false;
  }

  // printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
  //         drone1.pos.x, drone1.pos.y, drone1.vel.x, drone1.vel.y,
  //         drone2.pos.x, drone2.pos.y, drone2.vel.x, drone2.vel.y);
}


float pos_cmd[2] = {3.8, 0.8};

// #define FWD_VEL_CMD 0.2
#define KP_POS      0.2
void lateral_pos_ctrl(void) {

  struct NedCoor_f *pos_gps = stateGetPositionNed_f();
  dr_state.x = pos_gps->x;
  dr_state.y = pos_gps->y;

  struct NedCoor_f *vel_gps = stateGetSpeedNed_f();
  dr_state.vx = vel_gps->x;
  dr_state.vy = vel_gps->y;
  dr_state.yaw = stateGetNedToBodyEulers_f()->psi;

  // cmd pos = origin
	float curr_error_pos_w_x = (pos_cmd[0] - dr_state.x);
	float curr_error_pos_w_y = (pos_cmd[1] - dr_state.y);

  float curr_error_pos_x_velFrame =  cos(dr_state.yaw)*curr_error_pos_w_x + sin(dr_state.yaw)*curr_error_pos_w_y;
	float curr_error_pos_y_velFrame = -sin(dr_state.yaw)*curr_error_pos_w_x + cos(dr_state.yaw)*curr_error_pos_w_y;

  float vel_x_cmd_velFrame = curr_error_pos_x_velFrame * KP_POS; // FWD_VEL_CMD;
	float vel_y_cmd_velFrame = curr_error_pos_y_velFrame * KP_POS;

	vel_x_cmd_velFrame = bound_f(vel_x_cmd_velFrame, -MAX_VEL, MAX_VEL);
	vel_y_cmd_velFrame = bound_f(vel_y_cmd_velFrame, -MAX_VEL, MAX_VEL);

  if (!collision) {
    float yaw = atan2f(curr_error_pos_w_y, curr_error_pos_w_x);
    lateral_vel_ctrl(vel_x_cmd_velFrame, vel_y_cmd_velFrame);
    dr_cmd.yaw = yaw;
  }
}

#define KP_VEL      0.2
#define K_FF        0.0
#define MAX_BANK    0.5   // 26 deg max bank
void lateral_vel_ctrl(float velcmd_body_x, float velcmd_body_y) {

  float vel_x_est_velFrame =  cos(dr_state.yaw) * dr_state.vx + sin(dr_state.yaw) * dr_state.vy;
	float vel_y_est_velFrame = -sin(dr_state.yaw) * dr_state.vx + cos(dr_state.yaw) * dr_state.vy;

	float curr_error_vel_x = velcmd_body_x - vel_x_est_velFrame;
	float curr_error_vel_y = velcmd_body_y - vel_y_est_velFrame;

	float accx_cmd_velFrame = curr_error_vel_x * KP_VEL + K_FF * velcmd_body_x;
	float accy_cmd_velFrame = curr_error_vel_y * KP_VEL + K_FF * velcmd_body_y;

  dr_cmd.pitch = -1 * bound_f(accx_cmd_velFrame, -MAX_BANK, MAX_BANK);
  dr_cmd.roll  = bound_f(accy_cmd_velFrame, -MAX_BANK, MAX_BANK);  
}

void percevite_get_cmd(float *roll, float *pitch, float* yaw) {
  *roll  = (dr_cmd.roll);
  *pitch = (dr_cmd.pitch);
  *yaw   = (dr_cmd.yaw);
}
