package org.riverdell.robotics.utilities.managed.pidf

data class PIDFConfig(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kStatic: Double = 0.0,
)