package org.riverdell.robotics.autonomous.movement.purepursuit;

import org.jetbrains.annotations.NotNull;
import org.riverdell.robotics.autonomous.movement.PositionChangeAction;
import org.riverdell.robotics.autonomous.movement.PositionChangeAction;

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;

public class PurePursuitPositionChangeAction extends PositionChangeAction {
    public PurePursuitPositionChangeAction(@NotNull RootExecutionGroup executionGroup,
                                           @NotNull PurePursuitPath purePursuitPath) {
        super(null, executionGroup);
        withCustomPathAlgorithm(new PathAlgorithm(
                purePursuitPath::calculateTargetPose,
                (robotPose, targetPose) -> purePursuitPath.isFinished()
        ));
    }
}
