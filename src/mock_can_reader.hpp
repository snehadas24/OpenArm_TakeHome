#pragma once

#include "joint_state.hpp"

class MockCANReader {
public:
    explicit MockCANReader(double phase = 0.0);

    JointState read();

private:
    double t_;
    double phase_;
};