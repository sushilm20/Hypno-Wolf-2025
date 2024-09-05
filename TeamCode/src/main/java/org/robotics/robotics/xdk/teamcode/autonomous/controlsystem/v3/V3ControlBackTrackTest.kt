package org.robotics.robotics.xdk.teamcode.autonomous.controlsystem.v3

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.position.RobotStuckProtection
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateToPosition
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | BackTrack", group = "Test")
class V3ControlBackTrackTest : AbstractAutoPipeline(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { _, _ ->
        navigateToPosition(Pose(100.0, 0.0, 90.degrees)) {
            whenStuck(RobotStuckProtection(
                minimumRequiredRotationalDifference = 0.2,
                minimumMillisUntilDeemedStuck = 1500L,
                minimumRequiredTranslationalDifference = 0.1
            )) {
                backtrack(1)
            }
        }
    }
)