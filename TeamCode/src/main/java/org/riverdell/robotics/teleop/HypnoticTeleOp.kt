package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.liftgate.robotics.mono.Mono.commands
import org.riverdell.robotics.HypnoticRobot

@TeleOp(
    name = "Multiplayer",
    group = "Drive"
)
class HypnoticTeleOp : HypnoticRobot()
{
    private val gp1Commands by lazy { commands(gamepad1) }
    private val gp2Commands by lazy { commands(gamepad2) }

    override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands)
    override fun initialize()
    {

    }

    override fun opModeStart()
    {
        val robotDriver = GamepadEx(gamepad1)
        buildCommands()

        while (opModeIsActive())
        {
            val multiplier = 0.5 + gamepad1.right_trigger * 0.5
            drivetrain.driveRobotCentric(robotDriver, multiplier)
            runPeriodics()
        }
    }

    private fun buildCommands()
    {

    }
}