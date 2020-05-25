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
#ifndef PERCEVITE_VO_H
#define PERCEVITE_VO_H

/* for externed dr_data[MAX_DRONES] */
#include "modules/percevite_wifi/percevite_wifi.h"

#define RR  8.5
#define max(a,b)  ((a)>=(b)?(a):(b))
#define min(a,b)  ((a)<=(b)?(a):(b))

#define D2R (3.142/180.0)
#define R2D (180.0/3.142)
#define PI (3.141592)

/* MAX_VEL for control and velocity obstacle */
#define MAX_VEL 3.0

volatile bool percevite_requires_avoidance;

/* VO init: dummy init */
extern void percevite_vo_init(void);

/* VO periodic: detect and avoid */
extern void percevite_vo_periodic(void);

/* position control: cmd loop */
extern void lateral_pos_ctrl(void);
/* position control: lateral velocity control loop, shared by VO */
void lateral_vel_ctrl(float velcmd_body_x, float velcmd_body_y);
/* position control: copy and send command to outerloop */
void percevite_get_cmd(float *roll, float *pitch, float* yaw);

// dr_state is a copy of dr_data, used for position control
struct dr_state {
  float x;   // world pos x
  float y;   // world pos x
  float vx;  // world vel x
  float vy;  // world vel y
  float yaw; // yaw
};

// final struct holding commands to attitude outerloop
struct dr_cmd {
  float roll;      // body frame roll
  float pitch;    // body frame pitch -ve for fwd
  float yaw;      // body frame yaw
  float thrust;   // body frame command z force
};

// declare pos control and cmd structs
struct dr_state dr_state;
struct dr_cmd dr_cmd;

/* hero functions: velocity obstacle math */
void percevite_vo_resolve_by_project(const drone_data_t *robot1, float angle1, float angle2, float *centre, float *newvela);
bool percevite_vo_detect(const drone_data_t *robot1, const drone_data_t *robot2, float *resolution_cmd);

/* utils */
void polar2cart(float mag, float directn, float *cart);
void cart2polar(float *vel_vec, float *mag, float *head);
void calc_proj_matrix(const float *a, const float *b, float **c);

// bound a value to a range [min,max]
inline float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}

// scale angles to -180 - 180
inline float wrap_ang(float ang) {
	if (ang < -PI) {
		ang += 2 * PI;
	}
	if (ang > PI) {
		ang -= 2 * PI;
	}
	return ang;
}

/* quarantained hacks */
void vo_simulate_loop(drone_data_t* robot_sim);

#endif  // PERCEVITE_VO_H
