package org.riverdell.robotics

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxServoController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.ServoController
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import io.liftgate.robotics.mono.subsystem.System
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.Extension
import org.riverdell.robotics.subsystems.intake.Intake
import org.riverdell.robotics.subsystems.intake.IntakeV4B
import org.riverdell.robotics.utilities.hardware

abstract class HypnoticRobot(val opMode: LinearOpMode) : System
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticRobot
    }

    override val subsystems: MutableSet<Subsystem> = mutableSetOf()

    val drivetrain by lazy { Drivetrain(this) }
    val intake by lazy { Intake(this) }
    val intakeV4B by lazy { IntakeV4B(this) }
    val extension by lazy { Extension(this) }
/*
    val v4b by lazy { V4B(this) }
    val lift by lazy { Lift(this) }
    val extension by lazy { Extension(this) }
*/

    val multipleTelemetry by lazy {
        MultipleTelemetry(
            opMode.telemetry,
            FtcDashboard.getInstance().telemetry
        )
    }

    abstract fun initialize()
    abstract fun opModeStart()

    fun runPeriodics()
    {
        subsystems
            .map { it as AbstractSubsystem }
            .forEach { it.allPeriodic() }
    }

    open fun additionalSubSystems(): List<AbstractSubsystem>
    {
        return emptyList()
    }

    fun runOpMode()
    {
        instance = this

        val hardware = opMode.hardwareMap
            .getAll(LynxServoController::class.java)
            .firstOrNull()
        hardware?.initializeHardware()

        register(
            drivetrain, intake, intakeV4B, extension,
            *additionalSubSystems().toTypedArray()
        )

        // keep all log entries
        Mono.logSink = {
            multipleTelemetry.addLine("[mono] $it")
            multipleTelemetry.update()
            Log.i("mono", it)
        }

        initializeAll()
        initialize()

        opMode.waitForStart()
        if (opMode.isStopRequested)
        {
            return
        }

        startAll()
        opModeStart()
        disposeOfAll()
    }
}