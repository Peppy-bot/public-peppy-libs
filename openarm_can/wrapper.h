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

#ifdef __cplusplus
}
#endif
