package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "hors")
class Sushil : HypnoticAuto({ robot ->
    single("hors") {
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.Bar2)
        navigateTo(Pose(-1.5, -15.69, 46.degrees))
        robot.robot.outtake.setClawState(ClawState.Open)

        navigateTo(Pose(0.0, 0.0, 0.degrees))
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.Rest)
    }
})