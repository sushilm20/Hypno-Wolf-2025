package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.liftgate.robotics.mono.konfig.konfig
import kotlinx.serialization.Serializable
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNodeCollection

@TeleOp(
    name = "Konfig Test",
    group = "Tests"
)
class TestKonfig : LinearOpMode()
{
    @Serializable
    data class KonfigTestingClass(val test: String = "hii")

    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val test = konfig<NavigationNodeCollection>()

        while (opModeIsActive())
        {
            telemetry.addLine("Konfig value: ${test.get().nodes.keys}")
            telemetry.update()
            Thread.sleep(50L)
        }
    }
}