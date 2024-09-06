package org.riverdell.robotics.autonomous.movement.purepursuit

fun List<WaypointLike>.populateAndExtractActions() =
    toList()
        .onEachIndexed { index, waypointLike ->
            if (waypointLike is ActionWaypoint)
            {
                val prev = this@populateAndExtractActions.getOrNull(index - 1)
                if (prev is _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint)
                {
                    waypointLike.afterIndex = prev.id
                } else
                {
                    throw IllegalArgumentException(
                        "Cannot have two consecutive ActionWaypoints. Why the fuck are you doing it anyways? Merge it into one!"
                    )
                }
            }
        }
        .filterIsInstance<ActionWaypoint>()
        .associateBy { it.afterIndex }