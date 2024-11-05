package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(
    name = "Prepare V4B",
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

        val hardware = hardwareMap["intakeV4BLeft"] as Servo
        hardware.position = 1.0

        val right = hardwareMap["intakeV4BRight"] as Servo
        right.position = 0.0

        val coa = hardwareMap["intakeV4BCoaxial"] as Servo
        coa.position = 0.0

        while (opModeIsActive())
        {
            Thread.sleep(50L)
        }
    }
}