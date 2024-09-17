package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.AutonomousWrapper
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo

@Autonomous(name = "Test | Strafe", group = "Test")
class TestStrafe : AutonomousWrapper({ opMode ->
    single("strafe") {
        navigateTo(
            Pose(
                -15.0,
                -0.0,
                0.0
            )
        )
    }
})