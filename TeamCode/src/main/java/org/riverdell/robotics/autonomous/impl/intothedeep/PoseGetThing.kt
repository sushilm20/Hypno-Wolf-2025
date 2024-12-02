package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState

@Autonomous(name = "Pose Getter", group = "Test")
class PoseGetThing : HypnoticAuto({ opMode ->
    single("Intake") {
        opMode.robot.intakeComposite.prepareForPickup(WristState.Lateral, wideOpen = true)
        Thread.sleep(100000L)
    }
})