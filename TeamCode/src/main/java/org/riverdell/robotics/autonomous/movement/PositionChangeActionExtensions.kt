package org.riverdell.robotics.autonomous.movement

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.CubicBezierCurve
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.GuidedVectorFieldPositionChangeAction
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.Vector2D
import org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPositionChangeAction
import org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPath
import org.robotics.robotics.xdk.teamcode.autonomous.movement.purepursuit.WaypointLike

val Double.degrees: Double
    get() = Math.toRadians(this)

val Int.degrees: Double
    get() = Math.toRadians(this.toDouble())

@Deprecated(
    message = "Recommended to use new API navigateToPosition",
    replaceWith = ReplaceWith("navigateToPosition(pose, optBlock)")
)
fun RootExecutionGroup.navigateTo(pose: _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose, optBlock: _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction.() -> Unit = {}) =
    _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction(
        pose,
        this
    ).apply(optBlock).executeBlocking()

@Deprecated(
    message = "Recommended to use new API navigatePurePursuit",
    replaceWith = ReplaceWith("navigatePurePursuit(*waypoints, optBlock)")
)
fun RootExecutionGroup.purePursuitNavigateTo(
    vararg waypoints: WaypointLike,
    optBlock: _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction.() -> Unit = {}
) =
    _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPositionChangeAction(
        this,
        _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPath(*waypoints)
    ).apply(optBlock).executeBlocking()


fun cubicBezier(vararg waypoints: _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose): _root_ide_package_.org.riverdell.robotics.autonomous.movement.guidedvectorfield.CubicBezierCurve
{
    if (waypoints.size != 4)
    {
        throw IllegalArgumentException("Cubic Bezier Curve requires 4 points")
    }

    return _root_ide_package_.org.riverdell.robotics.autonomous.movement.guidedvectorfield.CubicBezierCurve(
        waypoints[0].toVector2D(), waypoints[1].toVector2D(),
        waypoints[2].toVector2D(), waypoints[3].toVector2D()
    )
}

fun _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose.toVector2D() =
    _root_ide_package_.org.riverdell.robotics.autonomous.movement.guidedvectorfield.Vector2D(x, y)

fun RootExecutionGroup.navigateUnstableGVF(
    curve: _root_ide_package_.org.riverdell.robotics.autonomous.movement.guidedvectorfield.CubicBezierCurve,
    configure: _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction.() -> Unit = {}
)
{
    val command =
        _root_ide_package_.org.riverdell.robotics.autonomous.movement.guidedvectorfield.GuidedVectorFieldPositionChangeAction(
            curve,
            this
        )
    command.configure()

    single("Unstable GVF navigation") {
        command.executeBlocking()
    }
}

fun RootExecutionGroup.navigatePurePursuit(
    vararg waypoints: WaypointLike,
    configure: _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction.() -> Unit = { }
)
{
    val command =
        _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPositionChangeAction(
            this,
            _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPath(
                *waypoints
            )
        )
    command.configure()

    single("PurePursuit navigation with ${waypoints.size} waypoints") {
        command.executeBlocking()
    }
}

fun RootExecutionGroup.navigateToPosition(
    pose: _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose,
    configure: _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction.() -> Unit = { }
)
{
    val command =
        _root_ide_package_.org.riverdell.robotics.autonomous.movement.PositionChangeAction(
            pose,
            this
        )
    command.configure()

    single("Position navigation to $pose") {
        command.executeBlocking()
    }
}