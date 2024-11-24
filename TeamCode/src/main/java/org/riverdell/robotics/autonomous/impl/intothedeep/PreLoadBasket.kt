package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo

@Autonomous(name = "Pre Load Basket", group = "Test")

class PreLoadBasket : HypnoticAuto({ opMode ->
    single("go to positions") {

        //navigateTo(Pose(0.0, 34.0, 0.degrees))

        navigateTo(Pose(-83.0, 28.0, 128.degrees))

        opMode.robot.intakeComposite.inToOut()
        opMode.robot.intakeComposite.initialOuttake()
        Thread.sleep(500)
        opMode.robot.intakeComposite.outtakeNext()
        opMode.robot.intakeComposite.outtakeNext()
        opMode.robot.intakeComposite.outtakeNext()
        Thread.sleep(1500)

        navigateTo(Pose(-81.0, 22.0, 128.degrees))
        opMode.robot.intakeComposite.outtakeCompleteAndRestSimple().join()
        Thread.sleep(500)

        navigateTo(Pose(-75.0, 27.0, 128.degrees))
        opMode.robot.intakeComposite.cancelOuttakeReadyToRest().join()
        Thread.sleep(200)

        navigateTo(Pose(-82.2, 28.6, 180.degrees))
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
    single("drop") {
        navigateTo(Pose(-83.0, 28.0, 128.degrees))

        opMode.robot.intakeComposite.inToOut()
        opMode.robot.intakeComposite.initialOuttake()
        Thread.sleep(500)
        opMode.robot.intakeComposite.outtakeNext()
        opMode.robot.intakeComposite.outtakeNext()
        opMode.robot.intakeComposite.outtakeNext()
        Thread.sleep(1500)

        navigateTo(Pose(-81.0, 22.0, 128.degrees))
        opMode.robot.intakeComposite.outtakeCompleteAndRestSimple().join()
        Thread.sleep(500)

        navigateTo(Pose(-77.0, 25.0, 128.degrees))
        opMode.robot.intakeComposite.cancelOuttakeReadyToRest().join()
        Thread.sleep(200)
    }
})