package org.riverdell.robotics.autonomous.movement.purepursuit

import org.riverdell.robotics.autonomous.movement.purepursuit.WaypointLike

data class ActionWaypoint(val action: () -> Unit) : WaypointLike
{
    var afterIndex = ""
    var hasExecuted = false
}