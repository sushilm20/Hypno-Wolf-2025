package org.riverdell.robotics.autonomous.movement.purepursuit

import kotlinx.serialization.Serializable
import kotlinx.serialization.Transient
import org.riverdell.robotics.autonomous.geometry.Point
import org.riverdell.robotics.autonomous.geometry.Pose
import java.util.UUID

@Serializable
class FieldWaypoint(
    val point: Point,
    val radius: Double
) : WaypointLike
{
    val type = if (point is Pose) Type.POSE else Type.POINT

    @Transient
    val id = UUID.randomUUID().toString()

    enum class Type
    {
        POINT,
        POSE
    }

    override fun toString() = "%s %s %.2f".format(type, point, radius)
}