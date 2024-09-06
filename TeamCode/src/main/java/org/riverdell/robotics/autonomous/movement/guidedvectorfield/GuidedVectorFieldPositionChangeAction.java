package org.riverdell.robotics.autonomous.movement.guidedvectorfield;

import org.jetbrains.annotations.NotNull;
import org.riverdell.robotics.autonomous.movement.PositionChangeAction;
import org.riverdell.robotics.autonomous.movement.PositionChangeAction;
import org.robotics.robotics.xdk.teamcode.autonomous.movement.purepursuit.PathAlgorithm;

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;

public class GuidedVectorFieldPositionChangeAction extends PositionChangeAction {
    public GuidedVectorFieldPositionChangeAction(
            @NotNull CubicBezierCurve cubicBezierCurve,
            @NotNull RootExecutionGroup executionGroup) {
        super(null, executionGroup);

        final GuidedVectorField vectorField = new GuidedVectorField(cubicBezierCurve);
        withCustomPathAlgorithm(new PathAlgorithm(
                vectorField::calculateGuidanceVector,
                (robotPose, targetPose) -> false,
                true
        ));
    }
}
