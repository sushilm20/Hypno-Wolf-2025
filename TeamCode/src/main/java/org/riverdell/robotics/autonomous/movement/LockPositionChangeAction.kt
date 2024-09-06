package org.riverdell.robotics.autonomous.movement

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.purepursuit.PathAlgorithm

class LockPositionChangeAction(
    lockPosition: Pose,
    unlockConsumer: (Pose, Pose) -> Boolean,
    executionGroup: RootExecutionGroup
) : PositionChangeAction(null, executionGroup)
{
    init
    {
        withAutomaticDeath(Long.MAX_VALUE.toDouble())
        withCustomPathAlgorithm(PathAlgorithm(
            { lockPosition },
            unlockConsumer
        ))
    }
}
