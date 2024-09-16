package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import io.liftgate.robotics.mono.subsystem.System
import org.riverdell.robotics.autonomous.impl.tests.ExampleSubSystem
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.Extension
import org.riverdell.robotics.subsystems.Intake
import org.riverdell.robotics.subsystems.Lift
import org.riverdell.robotics.subsystems.V4B

/**
 * A base implementation of a TeleOp. Contains lifecycles for
 * all subsystems and Mono gamepad command implementations.
 *
 * @author Subham
 * @since 9/5/2023
 */
abstract class AbstractTeleOp : LinearOpMode(), System
{
    override val subsystems: MutableSet<Subsystem> = mutableSetOf()

    private val gp1Commands by lazy { commands(gamepad1) }
    private val gp2Commands by lazy { commands(gamepad2) }

    private val drivetrain by lazy { Drivetrain(this) }
/*
    private val v4b by lazy { V4B(this) }
    private val intake by lazy { Intake(this) }
    private val lift by lazy { Lift(this) }
    private val extension by lazy { Extension(this) }*/
    private val test by lazy {
    ExampleSubSystem(this)
}

    abstract fun driveRobot(
        drivetrain: Drivetrain,
        driverOp: GamepadEx,
        multiplier: Double
    )

    override fun runOpMode()
    {
        register(
            test, drivetrain,
            gp1Commands, gp2Commands
        )

        val driverOp = GamepadEx(gamepad1)
        buildCommands()

        telemetry.addLine("Configured all subsystems. Waiting for start...")
        telemetry.update()

        initializeAll()
        waitForStart()

        telemetry.addLine("Initialized all subsystems. We're ready to go!")
        telemetry.update()

        while (opModeIsActive())
        {
            val multiplier = 0.5 + gamepad1.right_trigger * 0.5
            driveRobot(drivetrain, driverOp, multiplier)

            subsystems
                .map { it as AbstractSubsystem }
                .forEach { it.allPeriodic() }
        }

        disposeOfAll()
    }

    /**
     * Maps actions to gamepad buttons.
     */
    private fun buildCommands()
    {

    }
}