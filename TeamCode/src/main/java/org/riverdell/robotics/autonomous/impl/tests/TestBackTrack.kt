package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.AutonomousWrapper
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.RobotStuckProtection
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateToPosition

@Autonomous(name = "Test | BackTrack", group = "Test")
class TestBackTrack : AutonomousWrapper({ _, _ ->
    navigateToPosition(
        Pose(
            0.0,
            50.0,
            0.degrees
        )
    ) {
        whenStuck(RobotStuckProtection(
            minimumRequiredRotationalDifference = 0.5,
            minimumMillisUntilDeemedStuck = 1500L,
            minimumRequiredTranslationalDifference = 0.1
        )) {
            backtrack(1)
        }
    }
})