package org.riverdell.robotics.autonomous.movement

import kotlinx.serialization.Serializable

@Serializable
data class RobotStuckProtection(
    val minimumRequiredTranslationalDifference: Double = 0.5,
    val minimumRequiredRotationalDifference: Double = 0.05,
    val minimumMillisUntilDeemedStuck: Long = 2500L
)