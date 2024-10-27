package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import io.liftgate.robotics.mono.konfig.konfig
import kotlinx.serialization.Serializable
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNodeCollection
import org.riverdell.robotics.utilities.hardware

@TeleOp(
    name = "Prepare Four Bar Test",
    group = "Tests"
)
class PrepareFourBar : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val hardware = hardware<Servo>("intakeV4BLeft")
        hardware.position = 1.0

        val right = hardware<Servo>("intakeV4BRight")
        right.position = 0.0

        val coa = hardware<Servo>("intakeV4BCoaxial")
        coa.position = 0.0

        while (opModeIsActive())
        {
            Thread.sleep(50L)
        }
    }
}