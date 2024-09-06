package org.robotics.robotics.xdk.teamcode.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.position.RobotStuckProtection
import org.robotics.robotics.xdk.teamcode.autonomous.position.cubicBezier
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateToPosition
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateUnstableGVF
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile

@Autonomous(name = "Test | GVF", group = "Test")
class TestGVF : AbstractAutoPipeline(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { _, _ ->
        navigateUnstableGVF(cubicBezier(
          Pose(0.0, 20.0, 0.0),
          Pose(0.0, 30.0, 0.0),
          Pose(0.0, 40.0, 0.0),
          Pose(0.0, 50.0, 0.0),
        ))
    }
)