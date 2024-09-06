package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.movement.navigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | Strafe", group = "Test")
class TestStrafe : Autonomous(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { opMode, _ ->
        single("strafe") {
            navigateTo(
                _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(
                    -15.0,
                    -0.0,
                    0.0
                )
            )
        }
    }
)