package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo

@Autonomous(name = "Test | Move Backwards", group = "Test")
class TestMoveBackwards : HypnoticAuto({ opMode ->
    single("go to position") {
        navigateTo(Pose(0.0, 30.0, 90.degrees))
    }

    single("intake") {
        opMode.robot.intakeComposite
            .prepareForPickup()
            .join()

        Thread.sleep(500)
        opMode.robot.intakeComposite
            .intakeAndConfirm()
            .join()
        Thread.sleep(500L)

        opMode.robot.intakeComposite
            .confirmAndTransferAndReady()
            .join()
    }

})