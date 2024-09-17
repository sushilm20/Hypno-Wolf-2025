package org.riverdell.robotics.autonomous.movement.purepursuit

import kotlinx.serialization.Serializable
import kotlinx.serialization.Transient
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import java.util.UUID

@Serializable
data class PoseWaypoint(
    val pose: Pose,
    val radius: Double
) : WaypointLike
{
    @Transient
    override val id = UUID.randomUUID().toString()
}