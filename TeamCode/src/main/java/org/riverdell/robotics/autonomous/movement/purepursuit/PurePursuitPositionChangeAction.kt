package org.riverdell.robotics.autonomous.movement.purepursuit

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.movement.PositionChangeAction

class PurePursuitPositionChangeAction(
    executionGroup: RootExecutionGroup,
    purePursuitPath: PurePursuitPath
) : PositionChangeAction(null, executionGroup)
{
    init
    {
        withCustomPathAlgorithm(PathAlgorithm(
            { currentPose -> purePursuitPath.calculateTargetPose(currentPose) },
            { _, _ -> purePursuitPath.isFinished }
        ))
    }
}
