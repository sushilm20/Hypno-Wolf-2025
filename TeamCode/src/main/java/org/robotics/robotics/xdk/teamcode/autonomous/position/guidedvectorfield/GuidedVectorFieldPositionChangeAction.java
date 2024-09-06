package org.robotics.robotics.xdk.teamcode.autonomous.position.guidedvectorfield;

import org.jetbrains.annotations.NotNull;
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose;
import org.robotics.robotics.xdk.teamcode.autonomous.position.PositionChangeAction;
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.PathAlgorithm;
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.PurePursuitPath;

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;
import kotlin.jvm.functions.Function1;

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
