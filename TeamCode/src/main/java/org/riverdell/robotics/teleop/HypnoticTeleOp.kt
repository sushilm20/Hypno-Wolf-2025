package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import io.liftgate.robotics.mono.gamepad.bundle
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.utilities.hardware

@TeleOp(
    name = "Multiplayer",
    group = "Drive"
)
class HypnoticTeleOp : HypnoticRobot()
{
    private val gp1Commands by lazy { commands(gamepad1) }
    private val gp2Commands by lazy { commands(gamepad2) }

    val visionPipeline by lazy { VisionPipeline(this) }

    override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands, visionPipeline)
    override fun initialize()
    {
        while (!isStarted)
        {
            multipleTelemetry.addLine("Configured all subsystems. Waiting for start...")
            multipleTelemetry.update()
            runPeriodics()
        }
    }

    override fun opModeStart()
    {
        val robotDriver = GamepadEx(gamepad1)
        buildCommands()

        while (opModeIsActive())
        {
            val multiplier = 0.5 + gamepad1.right_trigger * 0.5
            drivetrain.driveRobotCentric(robotDriver, multiplier)

            gp1Commands.run()
            gp2Commands.run()

            runPeriodics()
        }
    }

    private fun buildCommands()
    {
        val wrist = hardware<Servo>("wrist")
        wrist.position = 0.5

        gp1Commands
            .where(ButtonType.ButtonX)
            .triggers {
                wrist.position = visionPipeline.sampleDetection.targetWristPosition
            }
            .repeatedlyWhilePressed()

        gp1Commands.doButtonUpdatesManually()
        gp2Commands.doButtonUpdatesManually()
    }
}