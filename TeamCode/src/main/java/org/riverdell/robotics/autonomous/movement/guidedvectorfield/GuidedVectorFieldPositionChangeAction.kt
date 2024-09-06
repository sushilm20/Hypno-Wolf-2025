package org.riverdell.robotics.autonomous.movement.guidedvectorfield

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.autonomous.movement.purepursuit.PathAlgorithm

class GuidedVectorFieldPositionChangeAction(
    cubicBezierCurve: CubicBezierCurve,
    executionGroup: RootExecutionGroup
) : PositionChangeAction(null, executionGroup)
{
    init
    {
        val vectorField = GuidedVectorField(cubicBezierCurve)
        withCustomPathAlgorithm(
            PathAlgorithm(
                { currentPose ->
                    vectorField.calculateGuidanceVector(currentPose)
                },
                { _, _ -> false },
                euclideanCompletionCheck = true
            )
        )
    }
}
