package org.riverdell.robotics.autonomous.movement.konfig

import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.AutonomousWrapper
import org.riverdell.robotics.autonomous.movement.geometry.CubicBezierCurve
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigatePurePursuit
import org.riverdell.robotics.autonomous.movement.navigateToPosition
import org.riverdell.robotics.autonomous.movement.navigateGVF
import org.riverdell.robotics.autonomous.movement.purepursuit.ActionWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.PoseWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.PositionWaypoint

abstract class KonfigAutonomous(
    private val konfigID: String,
) : AutonomousWrapper(root@{ wrapper ->
    val autonomousDefaults = wrapper.konfig<AutonomousDefaults> {
        withCustomFileID("autonomous-defaults")
    }

    val nodeCollection = wrapper.konfig<NavigationNodeCollection> {
        withCustomFileID(konfigID)
    }

    val parsed = nodeCollection.get().nodes.toList().associate {
        it.second to it.second.prepareForApplication(autonomousDefaults.get())
    }

    for ((node, nodeLines) in parsed)
    {
        if (node.navigationMode == NavigationMode.PurePursuit)
        {
            val mappedParsed = nodeLines.map {
                if (it.startsWith("*"))
                {
                    val reference = (wrapper as KonfigAutonomous).purePursuitActions[it.removePrefix("*")]
                        ?: throw IllegalArgumentException(
                            "No reference by $it was found to be replaced with an ActionWaypoint. Define it using the action purePursuitAction."
                        )

                    ActionWaypoint {
                        reference()
                    }
                } else if (it.contains("pose"))
                {
                    globalJson.decodeFromString<PoseWaypoint>(it)
                } else if (it.contains("point"))
                {
                    globalJson.decodeFromString<PositionWaypoint>(it)
                } else
                {
                    throw IllegalArgumentException(
                        "Illegal waypoint $it was given in the pure pursuit path."
                    )
                }
            }

            navigatePurePursuit(*mappedParsed.toTypedArray()) {
                node.applyToPositionChange(this@root, this)
            }
            continue
        }

        if (node.navigationMode == NavigationMode.GVF)
        {
            val curve = runCatching { globalJson.decodeFromString<CubicBezierCurve>(nodeLines.first()) }
                .getOrElse { throw IllegalArgumentException("Failed to parse GVF curve. It must be a CubicBezierCurve model as the first waypoint.") }

            navigateGVF(curve) {
                node.applyToPositionChange(this@root, this)
            }
            return@root
        }

        nodeLines.onEach {
            if (it.startsWith("*"))
            {
                val reference = (wrapper as KonfigAutonomous).actions[it.removePrefix("*")]
                    ?: throw IllegalArgumentException(
                        "No reference by $it was found to be replaced with an ActionWaypoint. Define it using the action purePursuitAction."
                    )

                reference()
                return@onEach
            }

            val pose = globalJson.decodeFromString<Pose>(it)
            navigateToPosition(pose) {
                node.applyToPositionChange(this@root, this)
            }
        }
    }
})
{
    private val actions = mutableMapOf<String, RootExecutionGroup.() -> Unit>()
    fun action(id: String, action: RootExecutionGroup.() -> Unit)
    {
        actions[id] = action
    }

    private val purePursuitActions = mutableMapOf<String, () -> Unit>()
    fun purePursuitAction(id: String, action: () -> Unit)
    {
        purePursuitActions[id] = action
    }

    abstract fun definitions()
}