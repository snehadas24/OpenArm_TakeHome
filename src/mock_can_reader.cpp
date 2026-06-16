#include "mock_can_reader.hpp"
#include "joint_state.hpp"

#include <chrono>
#include <cmath>

MockCANReader::MockCANReader(double phase)
    : t_(0.0), phase_(phase) {}

JointState MockCANReader::read() {
    t_ += 0.01;  // 100 Hz

    JointState state;

    state.timestamp =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

    state.position = std::sin(t_ + phase_);
    state.velocity = std::cos(t_ + phase_);
    state.torque = 0.2 * std::sin(2.0 * (t_ + phase_));

    return state;
}