package org.riverdell.robotics

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import io.liftgate.robotics.mono.subsystem.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.IMUProxySubsystem
import org.riverdell.robotics.subsystems.slides.Lift
import org.riverdell.robotics.subsystems.intake.composite.CompositeInteraction
import org.riverdell.robotics.subsystems.outtake.Sushil
import org.riverdell.robotics.teleop.HypnoticTeleOp

abstract class HypnoticRobot(val opMode: HypnoticOpMode) : System
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticRobot

        var resetMode = false
    }

    override val subsystems: MutableSet<Subsystem> = mutableSetOf()
    lateinit var hardware: HypnoticRobotHardware

    val drivetrain by lazy { Drivetrain(this) }
    val intakeComposite by lazy { CompositeInteraction(this) }
    val lift by lazy { Lift(this) }
    val outtake by lazy { Sushil(this) }

    val imuProxy by lazy { IMUProxySubsystem(opMode) }

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

    fun Telemetry.addEssentialLines()
    {
        addLine("Composite State: ${intakeComposite.state}")
        addLine("Claw State: ${opMode.robot.outtake.clawState}")
        addLine("Pivot State: ${opMode.robot.outtake.pivotState}")
        addLine("Wrist State: ${opMode.robot.outtake.wristState}")
        addLine("Wrist State: ${opMode.robot.intakeComposite.outtakeLevel}")

        addLine("LIFT Left Position: ${hardware.liftMotorLeft.currentPosition}")
        addLine("LIFT Left Power: ${hardware.liftMotorLeft.power}")
        addLine("LIFT Right Position: ${hardware.liftMotorRight.currentPosition}")
        addLine("LIFT Right Power: ${hardware.liftMotorRight.power}")
    }
    
    open fun additionalSubSystems(): List<AbstractSubsystem>
    {
        return emptyList()
    }

    fun runOpMode()
    {
        instance = this

        resetMode = opMode.gamepad1.a

        hardware = HypnoticRobotHardware(opMode)
        hardware.initializeHardware()

        register(
            drivetrain, outtake, lift,
            *additionalSubSystems().toTypedArray()
        )

        if (this is HypnoticTeleOp.TeleOpRobot)
        {
            register(imuProxy)
        }

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