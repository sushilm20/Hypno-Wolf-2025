package org.robotics.robotics.xdk.teamcode.autonomous

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.robotics.robotics.xdk.teamcode.autonomous.detection.TapeSide
import org.robotics.robotics.xdk.teamcode.autonomous.detection.TeamColor
import org.robotics.robotics.xdk.teamcode.autonomous.detection.VisionPipeline
import org.robotics.robotics.xdk.teamcode.autonomous.localizer.TwoWheelLocalizer
import org.robotics.robotics.xdk.teamcode.autonomous.profiles.AutonomousProfile
import org.robotics.robotics.xdk.teamcode.subsystem.Drivebase
import org.robotics.robotics.xdk.teamcode.subsystem.Elevator
import org.robotics.robotics.xdk.teamcode.subsystem.claw.ExtendableClaw
import org.robotics.robotics.xdk.teamcode.subsystem.drone.DroneLauncher
import kotlin.concurrent.thread

abstract class AbstractAutoPipeline(
    private val autonomousProfile: AutonomousProfile,
    private val teamColor: TeamColor = autonomousProfile.teamColor,
    internal val blockExecutionGroup: RootExecutionGroup.(AbstractAutoPipeline, TapeSide) -> Unit =
        autonomousProfile.buildExecutionGroup()
) : LinearOpMode(), io.liftgate.robotics.mono.subsystem.System
{
    companion object
    {
        @JvmStatic
        lateinit var instance: AbstractAutoPipeline
    }

    override val subsystems = mutableSetOf<Subsystem>()

    lateinit var frontRight: DcMotor
    lateinit var frontLeft: DcMotor

    lateinit var backRight: DcMotor
    lateinit var backLeft: DcMotor

    val drivebase by lazy { Drivebase(this) }

    internal val elevatorSubsystem by lazy { Elevator(this) }
    internal val clawSubsystem by lazy { ExtendableClaw(this) }

    private val airplaneSubsystem by lazy { DroneLauncher(this) }
    val visionPipeline by lazy { VisionPipeline(teamColor, this) }

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
        TwoWheelLocalizer(this@AbstractAutoPipeline)
    }

    override fun runOpMode()
    {
        instance = this

        register(
            clawSubsystem,
            elevatorSubsystem,
            airplaneSubsystem,
            drivebase,
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
            multipleTelemetry.addLine("Auto in initialized")
            multipleTelemetry.addData("Tape side", visionPipeline.getTapeSide())

            multipleTelemetry.addLine("=== PID Tuning Graph Outputs ===")

            /**
             *             drivetrain.getMultipleTelemetry().addData("Current Pose Y", robotPose.y);
             *             drivetrain.getMultipleTelemetry().addData("Current Pose X", robotPose.x);
             *             drivetrain.getMultipleTelemetry().addData("Current Pose Heading", robotPose.heading);
             *
             *             drivetrain.getMultipleTelemetry().addData("Power for Y", powers.y);
             *             drivetrain.getMultipleTelemetry().addData("Power for X", powers.y);
             *             drivetrain.getMultipleTelemetry().addData("Power for Turn", powers.heading);
             */
            /*multipleTelemetry.addData("Error", 0.0)
            multipleTelemetry.addData("Target", 0.0)
            multipleTelemetry.addData("Input", 0.0)
            multipleTelemetry.addData("Output", 0.0)
            multipleTelemetry.addData("Velocity", 0.0)*/

            multipleTelemetry.addData("Target Pose Y", 0.0)
            multipleTelemetry.addData("Target Pose X", 0.0)
            multipleTelemetry.addData("Target Pose Heading", 0.0)

            multipleTelemetry.addData("Current Pose Y", 0.0)
            multipleTelemetry.addData("Current Pose X", 0.0)
            multipleTelemetry.addData("Current Pose Heading", 0.0)
            multipleTelemetry.addData("Power for Y", 0.0)
            multipleTelemetry.addData("Power for X", 0.0)
            multipleTelemetry.addData("Power for Turn", 0.0)
//            multipleTelemetry.addData("Prev. Loop Time", 0)

            runCatching {
                multipleTelemetry.addData(
                    "IMU",
                    drivebase.getIMUYawPitchRollAngles()
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

        val tapeSide = visionPipeline.getTapeSide()
        thread {
            while (!isStopRequested)
            {
                subsystems.map { it as AbstractSubsystem }.forEach { it.allPeriodic() }
            }
        }

        thread {
            while (!isStopRequested)
            {
                voltage = hardwareMap.voltageSensor.first().voltage
                Thread.sleep(50L)
            }
        }

        thread {
            while (!isStopRequested)
            {
                localizer.update()
            }
        }

        val executionGroup = Mono.buildExecutionGroup {
            blockExecutionGroup(
                this@AbstractAutoPipeline, tapeSide
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
        drivebase.allDriveBaseMotors.forEach(consumer::invoke)
    }
}