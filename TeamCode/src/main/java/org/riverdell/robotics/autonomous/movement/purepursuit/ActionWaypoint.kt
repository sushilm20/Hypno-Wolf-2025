package org.riverdell.robotics.autonomous.movement.purepursuit

import java.util.UUID

data class ActionWaypoint(val action: () -> Unit) : WaypointLike
{
    var afterIndex = ""
    var hasExecuted = false

    override val id = UUID.randomUUID().toString()
}