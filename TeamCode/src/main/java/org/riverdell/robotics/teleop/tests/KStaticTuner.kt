package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.absoluteValue

@TeleOp(
    name = "kStatic Tuner",
    group = "Tests"
)
class KStaticTuner : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val extensionMotorLeft = hardwareMap["liftLeft"] as DcMotorEx
        extensionMotorLeft.direction = DcMotorSimple.Direction.REVERSE
        extensionMotorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val extensionMotorRight = hardwareMap["liftRight"] as DcMotorEx
        extensionMotorRight.direction = DcMotorSimple.Direction.FORWARD
        extensionMotorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        var kStaticFound = false
        var attemptNumber = 0
        val kStaticRecordings = mutableListOf<Double>()
        while (opModeIsActive())
        {
            if (kStaticFound)
            {
                Thread.sleep(50L)
                continue
            }

            if (((extensionMotorRight.velocity + extensionMotorLeft.velocity) / 2.0).absoluteValue < 50)
            {
                extensionMotorRight.power -= 0.001
                extensionMotorLeft.power -= 0.001
                telemetry.addLine(
                    "Finding kStatic #$attemptNumber... (${
                        extensionMotorLeft.currentPosition
                    })"
                )
                telemetry.update()
            } else
            {
                val targetPower = extensionMotorRight.power
                extensionMotorRight.power = 0.0
                extensionMotorLeft.power = 0.0

                Thread.sleep(1000L)

                attemptNumber += 1
                if (attemptNumber == 4)
                {
                    kStaticFound = true
                    telemetry.addLine(
                        "Minimum power to overcome static friction: ${
                            kStaticRecordings.average()
                        }"
                    )
                    telemetry.update()
                } else
                {
                    kStaticRecordings += targetPower
                }
            }
        }
    }
}