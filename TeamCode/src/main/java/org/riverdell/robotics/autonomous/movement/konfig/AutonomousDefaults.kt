package org.riverdell.robotics.autonomous.movement.konfig

import kotlinx.serialization.Serializable
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose

@Serializable
data class AutonomousDefaults(
    val poses: Map<String, Pose> = mapOf("pose1" to Pose(6.0, 9.0, 30.degrees)),
    val points: Map<String, Point> = mapOf("point1" to Point(6.0, 9.0))
)