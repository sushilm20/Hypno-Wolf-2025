package org.riverdell.robotics.autonomous.movement.konfig

import kotlinx.serialization.Serializable
import org.riverdell.robotics.autonomous.movement.geometry.Pose

@Serializable
data class AutonomousDefaults(
    val poses: Map<String, Pose> = mapOf(),
    val points: Map<String, Pose> = mapOf()
)