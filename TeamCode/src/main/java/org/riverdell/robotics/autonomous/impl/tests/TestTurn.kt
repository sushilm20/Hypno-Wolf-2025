package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.movement.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.movement.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | Turn", group = "Test")
class TestTurn : Autonomous(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { opMode, _ ->
        single("turn") {
            navigateTo(
                _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(
                    -0.0,
                    -0.0,
                    90.degrees
                )
            )
        }
    }
)