package org.riverdell.robotics.autonomous.movement.purepursuit

data class ActionWaypoint(val action: () -> Unit) : WaypointLike
{
    var afterIndex = ""
    var hasExecuted = false
}