package org.robotics.robotics.xdk.teamcode.autonomous.position.purepursuit;

import org.jetbrains.annotations.NotNull;
import org.robotics.robotics.xdk.teamcode.autonomous.position.PositionChangeAction;
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.PathAlgorithm;
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.PurePursuitPath;

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
