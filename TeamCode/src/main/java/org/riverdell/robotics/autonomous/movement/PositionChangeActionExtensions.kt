package org.riverdell.robotics.autonomous.movement

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.CubicBezierCurve
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.GuidedVectorFieldPositionChangeAction
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.Vector2D
import org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPositionChangeAction
import org.riverdell.robotics.autonomous.movement.purepursuit.PurePursuitPath
import org.riverdell.robotics.autonomous.movement.purepursuit.WaypointLike

val Double.degrees: Double
    get() = Math.toRadians(this)

val Int.degrees: Double
    get() = Math.toRadians(this.toDouble())

@Deprecated(
    message = "Recommended to use new API navigateToPosition",
    replaceWith = ReplaceWith("navigateToPosition(pose, optBlock)")
)
fun RootExecutionGroup.navigateTo(pose: Pose, optBlock: PositionChangeAction.() -> Unit = {}) =
    PositionChangeAction(
        pose,
        this
    ).apply(optBlock).executeBlocking()

@Deprecated(
    message = "Recommended to use new API navigatePurePursuit",
    replaceWith = ReplaceWith("navigatePurePursuit(*waypoints, optBlock)")
)
fun RootExecutionGroup.purePursuitNavigateTo(
    vararg waypoints: WaypointLike,
    optBlock: PositionChangeAction.() -> Unit = {}
) =
    PurePursuitPositionChangeAction(
        this,
        PurePursuitPath(*waypoints)
    ).apply(optBlock).executeBlocking()


fun cubicBezier(vararg waypoints: Pose): CubicBezierCurve
{
    if (waypoints.size != 4)
    {
        throw IllegalArgumentException("Cubic Bezier Curve requires 4 points")
    }

    return CubicBezierCurve(
        waypoints[0].toVector2D(), waypoints[1].toVector2D(),
        waypoints[2].toVector2D(), waypoints[3].toVector2D()
    )
}

fun Pose.toVector2D() =
    Vector2D(x, y)

fun RootExecutionGroup.lockToPosition(at: Pose, unlockConsumer: (Pose, Pose) -> Boolean)
{
    single("Locking at $at") {
        val command = LockPositionChangeAction(at, unlockConsumer, this@lockToPosition)
        command.executeBlocking()
    }
}

fun RootExecutionGroup.navigateUnstableGVF(
    curve: CubicBezierCurve,
    configure: PositionChangeAction.() -> Unit = {}
)
{


    single("Unstable GVF navigation") {

        val command =
            GuidedVectorFieldPositionChangeAction(
                curve,
                this@navigateUnstableGVF
            )
        command.configure()

        command.executeBlocking()
    }
}

fun RootExecutionGroup.navigatePurePursuit(
    vararg waypoints: WaypointLike,
    configure: PositionChangeAction.() -> Unit = { }
)
{


    single("PurePursuit navigation with ${waypoints.size} waypoints") {
        val command =
            PurePursuitPositionChangeAction(
                this@navigatePurePursuit,
                PurePursuitPath(
                    *waypoints
                )
            )
        command.configure()

        command.executeBlocking()
    }
}

fun RootExecutionGroup.navigateToPosition(
    pose: Pose,
    configure: PositionChangeAction.() -> Unit = { }
)
{
    single("Position navigation to $pose") {
        val command =
            PositionChangeAction(
                pose,
                this@navigateToPosition
            )
        command.configure()

        command.executeBlocking()
    }
}