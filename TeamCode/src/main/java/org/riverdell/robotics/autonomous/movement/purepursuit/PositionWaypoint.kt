package org.riverdell.robotics.autonomous.movement.purepursuit

import kotlinx.serialization.Serializable
import kotlinx.serialization.Transient
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import java.util.UUID

@Serializable
data class PositionWaypoint(
    val point: Point,
    val radius: Double
) : WaypointLike
{
    @Transient
    override val id = UUID.randomUUID().toString()
}