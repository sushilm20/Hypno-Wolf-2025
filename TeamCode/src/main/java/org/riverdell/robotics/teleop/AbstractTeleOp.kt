package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import io.liftgate.robotics.mono.Mono.commands
import org.riverdell.robotics.subsystems.Drivetrain

/**
 * A base implementation of a TeleOp. Contains lifecycles for
 * all subsystems and Mono gamepad command implementations.
 *
 * @author Subham
 * @since 9/5/2023
 */
abstract class AbstractTeleOp : AbstractLinearOpMode()
{
    private val gp1Commands by lazy { commands(gamepad1) }
    private val gp2Commands by lazy { commands(gamepad2) }

    abstract fun driveRobot(
        drivetrain: Drivetrain,
        driverOp: GamepadEx,
        multiplier: Double
    )

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
            driveRobot(drivetrain, robotDriver, multiplier)
            runPeriodics()
        }
    }

    private fun buildCommands()
    {

    }
}