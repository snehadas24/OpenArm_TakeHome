#pragma once
#include "joint_state.hpp"

class JointStateSource {
public:
    virtual ~JointStateSource() = default;
    virtual JointState read() = 0;
};