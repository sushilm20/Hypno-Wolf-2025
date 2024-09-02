package org.robotics.robotics.xdk.teamcode.autonomous.position

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.PurePursuitPath
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.WaypointLike

val Double.degrees: Double
    get() = Math.toRadians(this)

val Int.degrees: Double
    get() = Math.toRadians(this.toDouble())

@Deprecated(
    message = "Recommended to use new API navigateToPosition",
    replaceWith = ReplaceWith("navigateToPosition(pose, optBlock)")
)
fun RootExecutionGroup.navigateTo(pose: Pose, optBlock: PositionCommand.() -> Unit = {}) =
    PositionCommand(pose, this).apply(optBlock).executeBlocking()

@Deprecated(
    message = "Recommended to use new API navigatePurePursuit",
    replaceWith = ReplaceWith("navigatePurePursuit(*waypoints, optBlock)")
)
fun RootExecutionGroup.purePursuitNavigateTo(vararg waypoints: WaypointLike, optBlock: PositionCommand.() -> Unit = {}) =
    PurePursuitCommand(this, PurePursuitPath(*waypoints)).apply(optBlock).executeBlocking()

fun RootExecutionGroup.navigatePurePursuit(
    vararg waypoints: WaypointLike,
    configure: PositionCommand.() -> Unit = { }
)
{
    val command = PurePursuitCommand(this, PurePursuitPath(*waypoints))
    command.configure()

    single("PurePursuit navigation with ${waypoints.size} waypoints") {
        command.executeBlocking()
    }
}

fun RootExecutionGroup.navigateToPosition(
    pose: Pose,
    configure: PositionCommand.() -> Unit = { }
)
{
    val command = PositionCommand(pose, this)
    command.configure()

    single("Position navigation to $pose") {
        command.executeBlocking()
    }
}