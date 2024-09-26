package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
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

//    val visionPipeline by lazy { VisionPipeline(this) } // TODO: new season

    override fun additionalSubSystems() = listOf<AbstractSubsystem>(/*visionPipeline*/)
    override fun initialize()
    {
        instance = this

        while (opModeInInit())
        {
            multipleTelemetry.addLine("--- Initialization ---")

            runPeriodics()
            drivetrain.localizer.update()

            multipleTelemetry.clearAll()
            multipleTelemetry.addData(
                "Voltage",
                drivetrain.voltage()
            )
            multipleTelemetry.addData(
                "IMU",
                drivetrain.imu().getYaw(AngleUnit.DEGREES)
            )

            multipleTelemetry.update()
        }
    }

    override fun opModeStart()
    {
        thread {
            while (!isStopRequested)
            {
                runPeriodics()
                drivetrain.localizer.update()
            }
        }

        val executionGroup = Mono.buildExecutionGroup {
            blockExecutionGroup(
                this@HypnoticAuto
            )
        }

        executionGroup.executeBlocking()
    }
}
