package org.riverdell.robotics.autonomous.movement.purepursuit

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import kotlin.math.abs

class PurePursuitPath(vararg waypointLikes: WaypointLike)
{
    private val fieldWaypoints = waypointLikes.filter { it is PoseWaypoint || it is PositionWaypoint }
    private val actionWaypoints = waypointLikes.toList().populateAndExtractActions()
    private var currentWaypointIndex = 1

    var isFinished = false

    init
    {
        require(waypointLikes.size >= 2)
        if (fieldWaypoints.isEmpty() || (fieldWaypoints.last() !is PoseWaypoint))
        {
            throw IllegalArgumentException("Last waypoint is not a pose")
        }
    }

    fun calculateTargetPose(robot: Pose): Pose
    {
        if (isFinished)
        {
            return endPose()
        }

        val prev = fieldWaypoints[currentWaypointIndex - 1]

        val incomplete = actionWaypoints[prev.id]
        if (incomplete != null)
        {
            incomplete.action.invoke()
            incomplete.hasExecuted = true
            return robot
        }

        val target = fieldWaypoints[currentWaypointIndex]
        val point = if (target is PoseWaypoint)
        {
            target.pose
        } else if (target is PositionWaypoint)
        {
            target.point
        } else
        {
            throw IllegalArgumentException("What?")
        }

        val radius = if (target is PoseWaypoint)
        {
            target.radius
        } else if (target is PositionWaypoint)
        {
            target.radius
        } else
        {
            throw IllegalArgumentException("What?")
        }

        val distance = robot.distanceTo(point)

        return if (distance > radius)
        {
            val intersection = PurePursuitUtil.lineCircleIntersection(
                point, point, robot, radius)

            if (target is PoseWaypoint)
            {
                Pose(intersection, target.pose.heading)
            } else
            {
                val robotAngle = AngleUnit.normalizeRadians(robot.heading)
                val forwardAngle = intersection.subtract(robot).atan() - Math.PI / 2
                val backwardsAngle = AngleUnit.normalizeRadians(forwardAngle + Math.PI)
                val autoAngle = if (abs(AngleUnit.normalizeRadians(robotAngle - forwardAngle)) < abs(AngleUnit.normalizeRadians(robotAngle - backwardsAngle))) forwardAngle else backwardsAngle
                Pose(intersection, autoAngle)
            }
        } else
        {
            if (currentWaypointIndex == fieldWaypoints.size - 1)
            {
                isFinished = true
                endPose()
            } else
            {
                currentWaypointIndex++
                calculateTargetPose(robot)
            }
        }
    }

    private fun endPose(): Pose
    {
        val last = fieldWaypoints.last()
        return (last as PoseWaypoint).pose
    }
}