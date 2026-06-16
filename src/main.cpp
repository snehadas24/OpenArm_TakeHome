#include <chrono>
#include <iostream>
#include <thread>

#include "mock_can_reader.hpp"

int main() {

    MockCANReader motor1(0.0);
    MockCANReader motor2(1.57);

    std::cout << "Starting simulated CAN stream..." << std::endl;

    while (true) {

        JointState j1 = motor1.read();
        JointState j2 = motor2.read();

        std::cout
            << "Joint 1 | "
            << "pos: " << j1.position
            << " vel: " << j1.velocity
            << " tau: " << j1.torque
            << std::endl;

        std::cout
            << "Joint 2 | "
            << "pos: " << j2.position
            << " vel: " << j2.velocity
            << " tau: " << j2.torque
            << std::endl;

        std::cout << "---------------------" << std::endl;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)
        );
    }

    return 0;
}