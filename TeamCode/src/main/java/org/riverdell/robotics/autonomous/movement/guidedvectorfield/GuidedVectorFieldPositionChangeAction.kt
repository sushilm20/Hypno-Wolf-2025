package org.riverdell.robotics.autonomous.movement.guidedvectorfield

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.autonomous.movement.geometry.CubicBezierCurve

class GuidedVectorFieldPositionChangeAction(
    cubicBezierCurve: CubicBezierCurve,
    executionGroup: RootExecutionGroup
) : PositionChangeAction(null, executionGroup)
{
    init
    {
        val vectorField = EnhancedGuidedVectorField(cubicBezierCurve)
        withCustomPathAlgorithm(
            PathAlgorithm(
                { currentPose ->
                    vectorField.calculateNextPose(currentPose)
                },
                { _, _ -> false },
                euclideanCompletionCheck = true
            )
        )

        disableAutomaticDeath()
    }
}
