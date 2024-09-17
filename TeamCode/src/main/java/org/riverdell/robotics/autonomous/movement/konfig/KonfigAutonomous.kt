package org.riverdell.robotics.autonomous.movement.konfig

import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.AutonomousWrapper

abstract class KonfigAutonomous(
    private val konfigID: String,
) : AutonomousWrapper({ wrapper ->
    val autonomousDefaults = wrapper.konfig<AutonomousDefaults> {
        withCustomFileID("autonomous-defaults")
    }

    val nodeCollection = wrapper.konfig<NavigationNodeCollection> {
        withCustomFileID(konfigID)
    }
})
{
    private val actions = mutableMapOf<String, RootExecutionGroup.() -> Unit>()
    fun action(id: String, action: RootExecutionGroup.() -> Unit)
    {
        actions[id] = action
    }

    abstract fun definitions()
}