#ifndef RUN_LOGGING_H
#define RUN_LOGGING_H

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../models/atmosphere.h"
#include "../models/state.h"

static FILE *trajectory_log_file = NULL;
static FILE *reentry_guidance_log_file = NULL;

static inline void build_reentry_guidance_path(const char *trajectory_path,
                                               char *guidance_path,
                                               size_t guidance_path_size) {
  const char *path_sep = strrchr(trajectory_path, '/');
  if (path_sep == NULL) {
    snprintf(guidance_path, guidance_path_size, "reentry_guidance.csv");
    return;
  }

  int dir_len = (int)(path_sep - trajectory_path + 1);
  snprintf(guidance_path, guidance_path_size, "%.*sreentry_guidance.csv",
           dir_len, trajectory_path);
}

static inline void init_run_logging(const char *trajectory_path) {
  if (trajectory_log_file != NULL) {
    fclose(trajectory_log_file);
    trajectory_log_file = NULL;
  }
  if (reentry_guidance_log_file != NULL) {
    fclose(reentry_guidance_log_file);
    reentry_guidance_log_file = NULL;
  }

  trajectory_log_file = fopen(trajectory_path, "w");
  if (trajectory_log_file == NULL) {
    printf("Warning: could not open trajectory log at %s\n", trajectory_path);
  } else {
    fprintf(trajectory_log_file, "t, current_mass, x, y, z, vx, vy, vz, "
                   "a_lift, est_x, est_y, est_z, est_vx, "
                   "est_vy, est_vz, "
                   "true_delta_1, true_delta_2, "
                   "est_delta_1, est_delta_2, u1, u2, u3, "
                   "true_q_w, true_q_x, true_q_y, true_q_z, "
                   "est_q_w, est_q_x, est_q_y, est_q_z, "
                   "gyro_error_pitch, gyro_error_yaw \n");
  }

  char guidance_path[4096];
  build_reentry_guidance_path(trajectory_path, guidance_path,
                              sizeof(guidance_path));
  reentry_guidance_log_file = fopen(guidance_path, "w");
  if (reentry_guidance_log_file == NULL) {
    printf("Warning: could not open reentry guidance log at %s\n",
           guidance_path);
  } else {
    fprintf(reentry_guidance_log_file,
            "t, a_cmd_x, a_cmd_y, a_cmd_z, "
            "a_total_est_x, a_total_est_y, a_total_est_z\n");
  }
}

static inline void close_run_logging(void) {
  if (trajectory_log_file != NULL) {
    fclose(trajectory_log_file);
    trajectory_log_file = NULL;
  }
  if (reentry_guidance_log_file != NULL) {
    fclose(reentry_guidance_log_file);
    reentry_guidance_log_file = NULL;
  }
}

static inline void write_trajectory_log_row(double t, double current_mass,
                                            state *true_state, state *est_state,
                                            atm_cond *true_atm_cond) {
  if (trajectory_log_file == NULL) {
    return;
  }

  cartvec v_rel_E = get_relative_wind_eci(true_state, true_atm_cond);
  double v_rel_mag = norm(v_rel_E);
  cartvec u_hat_E = zeros();
  if (v_rel_mag > 1e-10) {
    u_hat_E = sdivide(v_rel_E, v_rel_mag);
  }
  cartvec u_hat_B = eci_to_body(u_hat_E, true_state->q_EB);

  fprintf(trajectory_log_file,
      "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
      "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
      "%g, %g\n",
          t, current_mass, true_state->position.x, true_state->position.y,
          true_state->position.z, true_state->velocity.x,
          true_state->velocity.y, true_state->velocity.z, 0.0,
          est_state->position.x, est_state->position.y, est_state->position.z,
          est_state->velocity.x, est_state->velocity.y, est_state->velocity.z,
          true_state->delta_1, true_state->delta_2, est_state->delta_1,
      est_state->delta_2, u_hat_B.x, u_hat_B.y, u_hat_B.z,
      true_state->q_EB.w, true_state->q_EB.x, true_state->q_EB.y,
      true_state->q_EB.z, est_state->q_EB.w, est_state->q_EB.x,
      est_state->q_EB.y, est_state->q_EB.z, true_state->gyro_error.pitch,
      true_state->gyro_error.yaw);
}

static inline void write_reentry_guidance_log_row(double t, cartvec a_cmd_E,
                                                  cartvec a_total_est) {
  if (reentry_guidance_log_file == NULL) {
    return;
  }

  fprintf(reentry_guidance_log_file, "%g, %g, %g, %g, %g, %g, %g\n", t,
          a_cmd_E.x, a_cmd_E.y, a_cmd_E.z, a_total_est.x, a_total_est.y,
          a_total_est.z);
}

#endif
