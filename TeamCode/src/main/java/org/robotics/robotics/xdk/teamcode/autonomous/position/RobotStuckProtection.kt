package org.robotics.robotics.xdk.teamcode.autonomous.position

data class RobotStuckProtection(
    val minimumRequiredTranslationalDifference: Int,
    val minimumRequiredRotationalDifference: Int,
    val minimumMillisUntilDeemedStuck: Long
)