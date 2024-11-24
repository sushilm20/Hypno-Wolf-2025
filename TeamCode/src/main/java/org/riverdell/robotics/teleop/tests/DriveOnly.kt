package org.riverdell.robotics.teleop.tests

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticRobotHardware

@TeleOp(
    name = "Drive Only",
    group = "Tests"
)
class DriveOnly : LinearOpMode()
{
    override fun runOpMode()
    {
        val backLeft = Motor(hardwareMap, "backLeft")
        val backRight = Motor(hardwareMap, "backRight")
        val frontLeft = Motor(hardwareMap, "frontLeft")
        val frontRight = Motor(hardwareMap, "frontRight")

        val backingDriveBase = MecanumDrive(
            frontLeft, frontRight, backLeft, backRight
        )

        waitForStart()

        while (opModeIsActive())
        {
            val scaleFactor = 0.5 + gamepad1.right_trigger * 0.5
            telemetry.addLine("${
                hardwareMap.voltageSensor.first().voltage
            }")
            telemetry.update()

            backingDriveBase.driveRobotCentric(
                -gamepad1.left_stick_x * scaleFactor,
                -gamepad1.left_stick_y * scaleFactor,
                -gamepad1.right_stick_x * scaleFactor,
                true
            )
        }
    }
}