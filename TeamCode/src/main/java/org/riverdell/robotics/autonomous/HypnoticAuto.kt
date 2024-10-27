package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.autonomous.movement.konfig.NavigationConfig
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.HypnoticAuto.Companion
import java.util.concurrent.CountDownLatch
import kotlin.concurrent.thread

abstract class HypnoticAuto(
    internal val blockExecutionGroup: RootExecutionGroup.(HypnoticAuto) -> Unit
) : HypnoticOpMode()
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticAuto
    }

   inner class HypnoticAutoRobot : HypnoticRobot(this@HypnoticAuto)
   {
       val navigationConfig by lazy {
           konfig<NavigationConfig> {
               withCustomFileID("navigation")
           }
       }

       val visionPipeline by lazy { VisionPipeline(this@HypnoticAuto) } // TODO: new season

       override fun additionalSubSystems() = listOf<AbstractSubsystem>(visionPipeline)
       override fun initialize()
       {
           HypnoticAuto.instance = this@HypnoticAuto

           while (opModeInInit())
           {
               runPeriodics()
               drivetrain.localizer.update()

               multipleTelemetry.addLine("--- Initialization ---")
               multipleTelemetry.addData(
                   "Voltage",
                   drivetrain.voltage()
               )
               multipleTelemetry.addData(
                   "IMU",
                   drivetrain.imu().getYaw(AngleUnit.DEGREES)
               )
               multipleTelemetry.addData(
                   "Pose",
                   drivetrain.localizer.pose
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

                   multipleTelemetry.addLine("--- Autonomous ---")
                   multipleTelemetry.addData(
                       "Voltage",
                       drivetrain.voltage()
                   )
                   multipleTelemetry.addData(
                       "IMU",
                       drivetrain.imu().getYaw(AngleUnit.DEGREES)
                   )
                   multipleTelemetry.addData(
                       "Pose",
                       drivetrain.localizer.pose
                   )

                   multipleTelemetry.update()
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

    override fun buildRobot() = HypnoticAutoRobot()
}
