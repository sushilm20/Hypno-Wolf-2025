package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.movement.RobotStuckProtection
import org.robotics.robotics.xdk.teamcode.autonomous.movement.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.movement.navigateToPosition
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | BackTrack", group = "Test")
class TestBackTrack : Autonomous(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { _, _ ->
        navigateToPosition(
            _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(
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
    }
)