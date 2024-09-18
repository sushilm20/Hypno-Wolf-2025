package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.autonomous.movement.konfig.NavigationConfig
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer
import org.riverdell.robotics.HypnoticRobot
import java.util.concurrent.CountDownLatch
import kotlin.concurrent.thread

abstract class HypnoticAuto(
    internal val blockExecutionGroup: RootExecutionGroup.(HypnoticAuto) -> Unit
) : HypnoticRobot()
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticAuto
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
        TwoWheelLocalizer(this@HypnoticAuto)
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
                    drivetrain.imu().getYaw(AngleUnit.DEGREES)
                )
            }.onFailure {
                multipleTelemetry.addData("IMU", 0.0)
            }

            multipleTelemetry.update()
        }
    }

    override fun opModeStart()
    {
        var completedLatch = false
        val latch = CountDownLatch(1)
        thread {
            while (!isStopRequested)
            {
                voltage = hardwareMap.voltageSensor.first().voltage
                localizer.update()
                runPeriodics()

                if (!completedLatch)
                {
                    latch.countDown()
                }
            }
        }

        val executionGroup = Mono.buildExecutionGroup {
            blockExecutionGroup(
                this@HypnoticAuto
            )
        }

        latch.await()
        executionGroup.executeBlocking()
    }
}
