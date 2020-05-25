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

  if (!percevite_requires_avoidance) {
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
