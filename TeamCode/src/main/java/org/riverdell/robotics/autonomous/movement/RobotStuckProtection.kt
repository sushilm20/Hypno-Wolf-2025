package org.riverdell.robotics.autonomous.movement

data class RobotStuckProtection(
    val minimumRequiredTranslationalDifference: Double,
    val minimumRequiredRotationalDifference: Double,
    val minimumMillisUntilDeemedStuck: Long
)