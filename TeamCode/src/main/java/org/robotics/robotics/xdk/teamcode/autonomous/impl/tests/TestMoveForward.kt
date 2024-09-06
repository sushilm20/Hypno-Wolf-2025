package org.robotics.robotics.xdk.teamcode.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.position.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigatePurePursuit
import org.robotics.robotics.xdk.teamcode.autonomous.position.navigateToPosition
import org.robotics.robotics.xdk.teamcode.autonomous.position.purePursuitNavigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.robotics.robotics.xdk.teamcode.autonomous.purepursuit.FieldWaypoint

@Autonomous(name = "Test | Move Forward", group = "Test")
class TestMoveForward : AbstractAutoPipeline(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { opMode, _ ->
        navigateToPosition(Pose(0.0, 0.0, 0.0))
        navigatePurePursuit(
            FieldWaypoint(Pose(0.0, 0.0, 0.degrees), 15.0),
            FieldWaypoint(Pose(0.0, -40.0, 0.degrees), 15.0)
        )

        single("move forward and backward") {
            purePursuitNavigateTo(
                FieldWaypoint(Pose(0.0, 0.0, 0.degrees), 15.0),
                FieldWaypoint(Pose(0.0, -40.0, 0.degrees), 15.0),
            )
        }
    }
)