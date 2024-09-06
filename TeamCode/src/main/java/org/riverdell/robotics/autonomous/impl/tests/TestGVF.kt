package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.movement.cubicBezier
import org.robotics.robotics.xdk.teamcode.autonomous.movement.navigateUnstableGVF
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | GVF", group = "Test")
class TestGVF : Autonomous(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { _, _ ->
        navigateUnstableGVF(cubicBezier(
            _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(0.0, 20.0, 0.0),
            _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(0.0, 30.0, 0.0),
            _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(0.0, 40.0, 0.0),
            _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(0.0, 50.0, 0.0),
        ))
    }
)