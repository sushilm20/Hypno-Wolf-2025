package org.riverdell.robotics.autonomous.movement.geometry

import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.kotlin.extensions.geometry.angle
import kotlinx.serialization.Serializable
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.Vector2D
import java.util.Locale
import kotlin.math.cos
import kotlin.math.sin

@Serializable
class Pose : Point
{
    var heading: Double

    @JvmOverloads
    constructor(x: Double = 0.0, y: Double = 0.0, heading: Double = 0.0) : super(x, y)
    {
        this.heading = AngleUnit.normalizeRadians(heading)
    }

    constructor(p: Point, heading: Double) : this(p.x, p.y, heading)
    constructor(vec: Vector2d, heading: Double) : this(vec.x, vec.y, heading)
    constructor(ftcPose: AprilTagPoseFtc)
    {
        val heading = Math.toRadians(-ftcPose.yaw)
        x = ftcPose.x * cos(heading) - ftcPose.y * sin(heading)
        y = ftcPose.x * sin(heading) + ftcPose.y * cos(heading)
        this.heading = heading
    }

    fun set(other: Pose)
    {
        x = other.x
        y = other.y
        heading = other.heading
    }

    fun add(other: Pose): Pose
    {
        return Pose(x + other.x, y + other.y, heading + other.heading)
    }

    fun add(other: Vector2D): Pose
    {
        return Pose(x + other.x, y + other.y, heading + other.heading)
    }

    fun subtract(other: Pose): Pose
    {
        return Pose(x - other.x, y - other.y, AngleUnit.normalizeRadians(heading - other.heading))
    }

    fun divide(other: Pose): Pose
    {
        return Pose(x / other.x, y / other.y, heading / other.heading)
    }

    fun subt(other: Pose): Pose
    {
        return Pose(x - other.x, y - other.y, heading - other.heading)
    }

    fun toVec2D(): Vector2d
    {
        return Vector2d(x, y)
    }

    override fun toString(): String
    {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.3f", x, y, heading)
    }
}