package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.AutonomousWrapper
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo

@Autonomous(name = "Test | Move Backwards", group = "Test")
class TestMoveBackwards : AutonomousWrapper({ _, _ ->
    single("move backwards") {
        navigateTo(
            Pose(
                -0.0,
                15.0,
                0.0
            )
        )
    }
})