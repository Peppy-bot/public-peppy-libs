#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to an openarm::can::socket::OpenArm instance.
typedef void* OpenArmHandle;

OpenArmHandle openarm_create(const char* can_interface, bool enable_fd);
void openarm_destroy(OpenArmHandle h);

void openarm_init_arm_motors(OpenArmHandle h,
                             const uint8_t* motor_types,
                             const uint32_t* send_can_ids,
                             const uint32_t* recv_can_ids,
                             int count);

void openarm_enable_all(OpenArmHandle h);
void openarm_disable_all(OpenArmHandle h);
void openarm_recv_all(OpenArmHandle h, int first_timeout_us);
void openarm_refresh_all(OpenArmHandle h);
void openarm_set_callback_mode_all(OpenArmHandle h, int mode);

void openarm_arm_mit_control(OpenArmHandle h,
                             const double* kp,
                             const double* kd,
                             const double* q,
                             const double* dq,
                             const double* tau,
                             int count);

void openarm_arm_get_state(OpenArmHandle h,
                           double* positions,
                           double* velocities,
                           double* torques,
                           int count);

void openarm_init_gripper_motor(OpenArmHandle h,
                                uint8_t motor_type,
                                uint32_t send_can_id,
                                uint32_t recv_can_id);

// Initialise the gripper motor in an explicit control mode (ControlMode in
// openarm/damiao_motor/dm_motor_constants.hpp; 1=MIT, 4=POS_FORCE). The v2.0 pinch
// gripper uses POS_FORCE.
void openarm_init_gripper_motor_mode(OpenArmHandle h,
                                     uint8_t motor_type,
                                     uint32_t send_can_id,
                                     uint32_t recv_can_id,
                                     uint8_t control_mode);

void openarm_gripper_mit_control(OpenArmHandle h,
                                 double kp,
                                 double kd,
                                 double q,
                                 double dq,
                                 double tau);

// POS_FORCE gripper command: drive to motor angle `q` (rad) with an absolute speed
// limit `dq` (rad/s) and a torque-current limit `i` (per-unit, 0..1). Requires the
// motor to have been initialised in POS_FORCE mode.
void openarm_gripper_pos_force_control(OpenArmHandle h,
                                       double q,
                                       double dq,
                                       double i);

void openarm_gripper_get_state(OpenArmHandle h,
                               double* position,
                               double* velocity,
                               double* torque);

#ifdef __cplusplus
}
#endif
