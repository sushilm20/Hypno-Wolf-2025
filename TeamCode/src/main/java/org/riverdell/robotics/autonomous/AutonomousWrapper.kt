package org.riverdell.robotics.autonomous

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.autonomous.detection.TeamColor
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer
import org.riverdell.robotics.autonomous.movement.konfig.NavigationConfig
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.Extension
import org.riverdell.robotics.subsystems.Intake
import org.riverdell.robotics.subsystems.Lift
import org.riverdell.robotics.subsystems.V4B
import org.riverdell.robotics.utilities.hardware
import kotlin.concurrent.thread

abstract class AutonomousWrapper(
    internal val blockExecutionGroup: RootExecutionGroup.(AutonomousWrapper) -> Unit
) : LinearOpMode(), io.liftgate.robotics.mono.subsystem.System
{
    companion object
    {
        @JvmStatic
        lateinit var instance: AutonomousWrapper
    }

    override val subsystems = mutableSetOf<Subsystem>()

    lateinit var frontRight: DcMotor
    lateinit var frontLeft: DcMotor

    lateinit var backRight: DcMotor
    lateinit var backLeft: DcMotor

    val drivetrain by lazy { Drivetrain(this) }
    val v4b by lazy { V4B(this) }
    val intake by lazy { Intake(this) }
    val lift by lazy { Lift(this) }
    val extension by lazy { Extension(this) }

    val navigationConfig by lazy {
        konfig<NavigationConfig> {
            withCustomFileID("navigation")
        }
    }

    val visionPipeline by lazy { VisionPipeline(TeamColor.Red, this) } // TODO: new season
    var voltage: Double = 0.0
        private set

    val multipleTelemetry by lazy {
        MultipleTelemetry(
            this.telemetry,
            FtcDashboard.getInstance().telemetry
        )
    }

    val runTimer = ElapsedTime()
    val localizer by lazy {
        TwoWheelLocalizer(this@AutonomousWrapper)
    }

    override fun runOpMode()
    {
        instance = this

        register(
            drivetrain, intake, v4b, lift, extension,
            visionPipeline
        )

        telemetry.isAutoClear = false

        // keep all log entries
        Mono.logSink = {
            multipleTelemetry.addLine("[mono] $it")
            multipleTelemetry.update()
            Log.i("mono", it)
        }

        localizer

        frontLeft = hardware<DcMotor>("frontLeft")
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        frontRight = hardware<DcMotor>("frontRight")
        frontRight.direction = DcMotorSimple.Direction.FORWARD
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backLeft = hardware<DcMotor>("backLeft")
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backRight = hardware<DcMotor>("backRight")
        backRight.direction = DcMotorSimple.Direction.FORWARD
        backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        multipleTelemetry.addLine("Waiting for start. Started detection...")
        multipleTelemetry.update()
        multipleTelemetry.clearAll()

        initializeAll()
        stopAndResetMotors()

        while (opModeInInit())
        {
            multipleTelemetry.addLine("Auto initialization...")

            runCatching {
                multipleTelemetry.addData(
                    "IMU",
                    drivetrain.getIMUYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES)
                )
            }.onFailure {
                multipleTelemetry.addData("IMU", 0.0)
            }

            multipleTelemetry.update()
        }

        waitForStart()
        multipleTelemetry.clearAll()
        runTimer.reset()

        // protect against premature stops before we even start the execution group
        if (isStopRequested)
        {
            return
        }

        thread {
            while (!isStopRequested)
            {
                voltage = hardwareMap.voltageSensor.first().voltage
                localizer.update()

                subsystems.map { it as AbstractSubsystem }.forEach { it.allPeriodic() }
            }
        }

        val executionGroup = Mono.buildExecutionGroup {
            blockExecutionGroup(
                this@AutonomousWrapper
            )
        }

        runWithoutEncoders()
        executionGroup.executeBlocking()
        disposeOfAll()

        Mono.logSink = { }
    }

    fun stopAndResetMotors() = configureMotorsToDo {
        it.power = 0.0
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun runWithoutEncoders() = configureMotorsToDo {
        it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun configureMotorsToDo(consumer: (DcMotor) -> Unit)
    {
        listOf(backLeft, frontLeft, frontRight, backRight).forEach(consumer::invoke)
    }
}
