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
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
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
       val navigationConfig = NavigationConfig()
//       val visionPipeline by lazy { VisionPipeline(this@HypnoticAuto) } // TODO: new season

       override fun additionalSubSystems() = listOf<AbstractSubsystem>(/*visionPipeline*/)
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

               multipleTelemetry.addData(
                   "H Velocity Error",
                   0.0
               )

               multipleTelemetry.addData(
                   "X Velocity Error",
                   0.0
               )


               multipleTelemetry.addData(
                   "Y Velocity Error",
                   0.0
               )

               multipleTelemetry.addData(
                   "Period",
                   0.0
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

//                   multipleTelemetry.addData(
//                       "H Velocity Error",
//                       PositionChangeAction.hController.velocityError
//                   )
//
//                   multipleTelemetry.addData(
//                       "Y Velocity Error",
//                       PositionChangeAction.yController.velocityError
//                   )
//
                   multipleTelemetry.addData(
                       "X Velocity Error",
                       PositionChangeAction.xController.averageVelocity
                   )
                   multipleTelemetry.addData(
                       "Y Velocity Error",
                       PositionChangeAction.yController.averageVelocity
                   )
//
//                   multipleTelemetry.addData(
//                       "Period",
//                       PositionChangeAction.xController.period
//                   )

                   multipleTelemetry.update()
               }
           }

           thread {
               while (!isStopRequested) {
                   drivetrain.localizer.update()
               }
           }

           val executionGroup = Mono.buildExecutionGroup {
               blockExecutionGroup(
                   this@HypnoticAuto
               )
           }

           ManagedMotorGroup.keepEncoderPositions = true
           executionGroup.executeBlocking()
       }
   }

    override fun buildRobot() = HypnoticAutoRobot()
}
