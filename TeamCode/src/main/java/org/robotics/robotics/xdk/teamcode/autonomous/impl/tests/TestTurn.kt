package org.robotics.robotics.xdk.teamcode.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | Turn", group = "Test")
class TestTurn : AbstractAutoPipeline(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { opMode, _ ->
        single("turn") {
            navigateTo(Pose(-0.0, -0.0, 90.degrees))
        }
    }
)