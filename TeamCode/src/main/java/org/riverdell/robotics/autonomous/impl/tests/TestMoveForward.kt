package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.geometry.Pose
import org.robotics.robotics.xdk.teamcode.autonomous.movement.degrees
import org.robotics.robotics.xdk.teamcode.autonomous.movement.navigatePurePursuit
import org.robotics.robotics.xdk.teamcode.autonomous.movement.navigateToPosition
import org.robotics.robotics.xdk.teamcode.autonomous.movement.purePursuitNavigateTo
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint

@Autonomous(name = "Test | Move Forward", group = "Test")
class TestMoveForward : Autonomous(
    AutonomousProfile.RedPlayer1TwoPlusZero,
    blockExecutionGroup = { opMode, _ ->
        navigateToPosition(
            _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose(
                0.0,
                0.0,
                0.0
            )
        )
        navigatePurePursuit(
            _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint(
                Pose(0.0, 0.0, 0.degrees),
                15.0
            ),
            _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint(
                Pose(0.0, -40.0, 0.degrees),
                15.0
            )
        )

        single("move forward and backward") {
            purePursuitNavigateTo(
                _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint(
                    Pose(0.0, 0.0, 0.degrees),
                    15.0
                ),
                _root_ide_package_.org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint(
                    Pose(0.0, -40.0, 0.degrees),
                    15.0
                ),
            )
        }
    }
)