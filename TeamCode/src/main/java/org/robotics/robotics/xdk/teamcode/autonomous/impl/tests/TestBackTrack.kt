package org.robotics.robotics.xdk.teamcode.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.position.RobotStuckProtection
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateToPosition
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | BackTrack", group = "Test")
class TestBackTrack : AbstractAutoPipeline(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { _, _ ->
        navigateToPosition(Pose(0.0, 50.0, 0.degrees)) {
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