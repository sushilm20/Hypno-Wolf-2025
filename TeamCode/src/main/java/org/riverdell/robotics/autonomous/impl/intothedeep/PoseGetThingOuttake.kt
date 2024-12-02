package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "Pose Getter Outtake", group = "Test")
class PoseGetThingOuttake : HypnoticAuto({ opMode ->
    single("Outtake") {
        opMode.robot.intakeComposite
            .initialOuttakeFromRest(OuttakeLevel.HighBasket)

        Thread.sleep(100000L)
    }
})