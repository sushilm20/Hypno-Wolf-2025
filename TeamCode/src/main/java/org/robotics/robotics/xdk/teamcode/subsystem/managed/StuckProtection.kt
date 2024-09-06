package org.robotics.robotics.xdk.teamcode.subsystem.managed

data class StuckProtection(
    val minimumRequiredPositionDifference: Int,
    val timeStuckUnderMinimumMillis: Long
)