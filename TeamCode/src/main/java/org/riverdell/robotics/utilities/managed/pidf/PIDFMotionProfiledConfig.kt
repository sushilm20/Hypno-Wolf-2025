package org.riverdell.robotics.utilities.managed.pidf

import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints

@Serializable
data class PIDFMotionProfiledConfig(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kStatic: Double = 0.0,
    val motionProfileConstraints: MotionProfileConstraints = MotionProfileConstraints(0.0, 0.0, 0.0)
)