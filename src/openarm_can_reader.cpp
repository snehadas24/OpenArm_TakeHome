JointState OpenArmCANReader::read() {

    openarm.refresh_all();
    openarm.recv_all(300);

    auto& motor = openarm.get_arm().get_motors()[0];

    JointState state;

    state.timestamp = /* time */;
    state.position  = motor.get_position();
    state.velocity  = motor.get_velocity();
    state.torque    = motor.get_torque();

    return state;
}