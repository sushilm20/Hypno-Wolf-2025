package org.riverdell.robotics.utilities.managed.pidf

import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints

data class PIDFMotionProfiledConfig(
    var kP: Double = 0.01,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kStatic: Double = 0.0,
    var motionProfileConstraints: ProfileConstraints = ProfileConstraints(0.0, 0.0, 0.0)
)