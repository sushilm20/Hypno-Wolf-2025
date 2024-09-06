package org.riverdell.robotics.utilities.managed

data class StuckProtection(
    val minimumRequiredPositionDifference: Int,
    val timeStuckUnderMinimumMillis: Long
)