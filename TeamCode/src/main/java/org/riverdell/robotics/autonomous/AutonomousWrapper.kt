package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.autonomous.movement.konfig.NavigationConfig
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer
import org.riverdell.robotics.teleop.AbstractLinearOpMode
import kotlin.concurrent.thread

abstract class AutonomousWrapper(
    internal val blockExecutionGroup: RootExecutionGroup.(AutonomousWrapper) -> Unit
) : AbstractLinearOpMode()
{
    companion object
    {
        @JvmStatic
        lateinit var instance: AutonomousWrapper
    }

    val navigationConfig by lazy {
        konfig<NavigationConfig> {
            withCustomFileID("navigation")
        }
    }

    val visionPipeline by lazy { VisionPipeline(this) } // TODO: new season
    var voltage: Double = 0.0
        private set

    val localizer by lazy {
        TwoWheelLocalizer(this@AutonomousWrapper)
    }

    override fun additionalSubSystems() = listOf(visionPipeline)
    override fun initialize()
    {
        instance = this

        while (opModeInInit())
        {
            multipleTelemetry.addLine("--- Initialization ---")

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
    }

    override fun opModeStart()
    {
        thread {
            while (!isStopRequested)
            {
                voltage = hardwareMap.voltageSensor.first().voltage
                localizer.update()
                runPeriodics()
            }
        }

        val executionGroup = Mono.buildExecutionGroup {
            blockExecutionGroup(
                this@AutonomousWrapper
            )
        }

        executionGroup.executeBlocking()
    }
}
