package org.robotics.robotics.xdk.teamcode.subsystem.pidf

data class StuckProtection(
    val minimumRequiredPositionDifference: Int,
    val timeStuckUnderMinimumMillis: Long
)