package org.robotics.robotics.xdk.teamcode.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | Move Backwards", group = "Test")
class TestMoveBackwards : AbstractAutoPipeline(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { opMode, _ ->
        single("move backwards") {
            navigateTo(Pose(-0.0, 15.0, 0.0))
        }
    }
)