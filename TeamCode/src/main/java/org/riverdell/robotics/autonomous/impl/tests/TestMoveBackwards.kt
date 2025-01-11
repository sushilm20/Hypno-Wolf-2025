package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateTo

@Autonomous(name = "Test | Movement", group = "Test")
class TestMoveBackwards : HypnoticAuto({ opMode ->
    single("go to position") {
        navigateTo(Pose(TestConfig.x, TestConfig.y, TestConfig.turn.degrees)) {withAutomaticDeath(TestConfig.automaticDeath)}
    }
})