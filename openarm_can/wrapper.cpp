// C wrapper around openarm::can::socket::OpenArm (arm + gripper motors).
// Only compiled on Linux where libopenarm-can-dev is available.
#include "wrapper.h"

#include <cstdlib>
#include <iostream>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

using OA = openarm::can::socket::OpenArm;

extern "C" {

OpenArmHandle openarm_create(const char* can_interface, bool enable_fd) {
    try {
        auto* arm = new OA(std::string(can_interface), enable_fd);
        return static_cast<OpenArmHandle>(arm);
    } catch (const std::exception& e) {
        std::cerr << "openarm_create failed: " << e.what() << std::endl;
        return nullptr;
    }
}

void openarm_destroy(OpenArmHandle h) {
    if (!h) return;
    delete static_cast<OA*>(h);
}

void openarm_init_arm_motors(OpenArmHandle h,
                             const uint8_t* motor_types,
                             const uint32_t* send_can_ids,
                             const uint32_t* recv_can_ids,
                             int count) {
    if (!h) { std::cerr << "openarm_init_arm_motors: null handle" << std::endl; std::abort(); }
    auto* arm = static_cast<OA*>(h);
    std::vector<openarm::damiao_motor::MotorType> types;
    std::vector<uint32_t> send_ids, recv_ids;
    types.reserve(count);
    send_ids.reserve(count);
    recv_ids.reserve(count);
    for (int i = 0; i < count; ++i) {
        types.push_back(static_cast<openarm::damiao_motor::MotorType>(motor_types[i]));
        send_ids.push_back(send_can_ids[i]);
        recv_ids.push_back(recv_can_ids[i]);
    }
    arm->init_arm_motors(types, send_ids, recv_ids);
}

void openarm_enable_all(OpenArmHandle h) {
    if (!h) { std::cerr << "openarm_enable_all: null handle" << std::endl; std::abort(); }
    static_cast<OA*>(h)->enable_all();
}

void openarm_disable_all(OpenArmHandle h) {
    if (!h) { std::cerr << "openarm_disable_all: null handle" << std::endl; std::abort(); }
    static_cast<OA*>(h)->disable_all();
}

void openarm_recv_all(OpenArmHandle h, int first_timeout_us) {
    if (!h) { std::cerr << "openarm_recv_all: null handle" << std::endl; std::abort(); }
    static_cast<OA*>(h)->recv_all(first_timeout_us);
}

void openarm_refresh_all(OpenArmHandle h) {
    if (!h) { std::cerr << "openarm_refresh_all: null handle" << std::endl; std::abort(); }
    static_cast<OA*>(h)->refresh_all();
}

void openarm_set_callback_mode_all(OpenArmHandle h, int mode) {
    if (!h) { std::cerr << "openarm_set_callback_mode_all: null handle" << std::endl; std::abort(); }
    using CM = openarm::damiao_motor::CallbackMode;
    CM cm;
    switch (mode) {
        case 0:  cm = CM::STATE;  break;
        case 1:  cm = CM::PARAM;  break;
        case 2:  cm = CM::IGNORE; break;
        default:
            std::cerr << "openarm_set_callback_mode_all: invalid mode " << mode << std::endl;
            std::abort();
    }
    static_cast<OA*>(h)->set_callback_mode_all(cm);
}

void openarm_arm_mit_control(OpenArmHandle h,
                             const double* kp,
                             const double* kd,
                             const double* q,
                             const double* dq,
                             const double* tau,
                             int count) {
    if (!h) { std::cerr << "openarm_arm_mit_control: null handle" << std::endl; std::abort(); }
    auto* arm = static_cast<OA*>(h);
    std::vector<openarm::damiao_motor::MITParam> params;
    params.reserve(count);
    for (int i = 0; i < count; ++i) {
        params.push_back({kp[i], kd[i], q[i], dq[i], tau[i]});
    }
    arm->get_arm().mit_control_all(params);
}

void openarm_arm_get_state(OpenArmHandle h,
                           double* positions,
                           double* velocities,
                           double* torques,
                           int count) {
    if (!h) { std::cerr << "openarm_arm_get_state: null handle" << std::endl; std::abort(); }
    auto* arm = static_cast<OA*>(h);
    const auto& motors = arm->get_arm().get_motors();
    if (static_cast<int>(motors.size()) != count) {
        std::cerr << "openarm_arm_get_state: expected " << count
                  << " motors, got " << motors.size()
                  << " (init_motors not called?)" << std::endl;
        std::abort();
    }
    for (int i = 0; i < count; ++i) {
        positions[i]  = motors[i].get_position();
        velocities[i] = motors[i].get_velocity();
        torques[i]    = motors[i].get_torque();
    }
}

void openarm_init_gripper_motor(OpenArmHandle h,
                                uint8_t motor_type,
                                uint32_t send_can_id,
                                uint32_t recv_can_id) {
    if (!h) { std::cerr << "openarm_init_gripper_motor: null handle" << std::endl; std::abort(); }
    auto* arm = static_cast<OA*>(h);
    arm->init_gripper_motor(
        static_cast<openarm::damiao_motor::MotorType>(motor_type),
        send_can_id,
        recv_can_id);
}

void openarm_gripper_mit_control(OpenArmHandle h,
                                 double kp,
                                 double kd,
                                 double q,
                                 double dq,
                                 double tau) {
    if (!h) { std::cerr << "openarm_gripper_mit_control: null handle" << std::endl; std::abort(); }
    auto* arm = static_cast<OA*>(h);
    arm->get_gripper().mit_control_all({{kp, kd, q, dq, tau}});
}

void openarm_gripper_get_state(OpenArmHandle h,
                               double* position,
                               double* velocity,
                               double* torque) {
    if (!h) { std::cerr << "openarm_gripper_get_state: null handle" << std::endl; std::abort(); }
    auto* arm = static_cast<OA*>(h);
    const auto& motors = arm->get_gripper().get_motors();
    if (motors.empty()) {
        std::cerr << "openarm_gripper_get_state: gripper motor not initialized "
                     "(init_motor not called?)" << std::endl;
        std::abort();
    }
    *position = motors[0].get_position();
    *velocity = motors[0].get_velocity();
    *torque   = motors[0].get_torque();
}

} // extern "C"
