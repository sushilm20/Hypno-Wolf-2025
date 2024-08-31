package org.robotics.robotics.xdk.teamcode.autonomous.position

data class RobotStuckProtection(
    val minimumRequiredTranslationalDifference: Double,
    val minimumRequiredRotationalDifference: Double,
    val minimumMillisUntilDeemedStuck: Long
)