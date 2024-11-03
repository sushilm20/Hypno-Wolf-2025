package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.subsystems.intake.composite.IntakeCompositeState
import org.riverdell.robotics.subsystems.intake.composite.IntakeConfig
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min

@TeleOp(
    name = "Multiplayer",
    group = "Drive"
)
class HypnoticTeleOp : HypnoticOpMode()
{
    class TeleOpRobot(opMode: HypnoticOpMode) : HypnoticRobot(opMode)
    {
        private val gp1Commands by lazy { commands(opMode.gamepad1) }
        private val gp2Commands by lazy { commands(opMode.gamepad2) }

        val visionPipeline by lazy { VisionPipeline(opMode) }

        override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands /*visionPipeline*/)
        override fun initialize()
        {
            /*wrist.position = 0.5
            visionPipeline.sampleDetection.supplyCurrentWristPosition { wrist.position }
            visionPipeline.sampleDetection.setDetectionType(SampleType.Blue)*/

            multipleTelemetry.addLine("Configured all subsystems. Waiting for start...")
            multipleTelemetry.update()

            while (!opMode.isStarted)
            {
                runPeriodics()
            }
        }

        override fun opModeStart()
        {
            val robotDriver = GamepadEx(opMode.gamepad1)
            buildCommands()

            multipleTelemetry.addLine("Started!")
            multipleTelemetry.update()

            var loopTime: Long
            while (opMode.opModeIsActive())
            {
                loopTime = System.nanoTime()

                val multiplier = 0.5 + opMode.gamepad1.right_trigger * 0.5
                drivetrain.driveRobotCentric(robotDriver, multiplier)

                gp1Commands.run()
                gp2Commands.run()

                if (intakeComposite.state == IntakeCompositeState.Pickup)
                {
                    val wantedPower = -opMode.gamepad2.left_trigger + opMode.gamepad2.right_trigger
                    if (wantedPower.absoluteValue > 0.1)
                    {
                        if (wantedPower < 0)
                        {
                            if (extension.slides.currentPosition() < 5)
                            {
                                extension.slides.supplyPowerToAll(0.0)
                            } else
                            {
                                extension.slides.supplyPowerToAll(wantedPower.toDouble() / 2.0)
                            }
                        } else
                        {
                            if (extension.slides.currentPosition() >= IntakeConfig.MAX_EXTENSION)
                            {
                                extension.slides.supplyPowerToAll(0.0)
                            } else
                            {
                                extension.slides.supplyPowerToAll(wantedPower.toDouble() / 2.0)
                            }
                        }
                    }
                }

                runPeriodics()

                opMode.telemetry.addData("Loop Refresh Rate ", 1000000000 / (System.nanoTime() - loopTime).toDouble())

                opMode.telemetry.addLine("Intake State: ${intake.intakeState}")
                opMode.telemetry.addLine("Wrist State: ${intake.wristState}")
                opMode.telemetry.addLine("4BR State: ${intakeV4B.v4bState}")
                opMode.telemetry.addLine("Coaxial State: ${intakeV4B.coaxialState}")
                opMode.telemetry.addLine("Composite State: ${intakeComposite.state}")

                opMode.telemetry.addLine("Extendo Right (MASTER) Position: ${hardware.extensionMotorLeft.currentPosition}")
                opMode.telemetry.addLine("Extendo Right Current Draw: ${hardware.extensionMotorRight.getCurrent(CurrentUnit.AMPS)}")
                opMode.telemetry.update()
            }
        }

        private fun buildCommands()
        {
           gp2Commands.apply {
               where(ButtonType.DPadLeft)
                   .onlyWhen { intakeComposite.state == IntakeCompositeState.Pickup }
                   .triggers {
                       intake.perpendicularWrist()
                   }
                   .whenPressedOnce()

               where(ButtonType.DPadRight)
                   .onlyWhen { intakeComposite.state == IntakeCompositeState.Pickup }
                   .triggers {
                       intake.lateralWrist()
                   }
                   .whenPressedOnce()

               where(ButtonType.BumperRight)
                   .onlyWhen { intakeComposite.state == IntakeCompositeState.Pickup }
                   .triggers {
                       intake.dynamicWrist(
                           min(intake.currentDynamicPosition() + 0.05, 1.0)
                       )
                   }
                   .repeatedlyWhilePressed()

               where(ButtonType.BumperLeft)
                   .onlyWhen { intakeComposite.state == IntakeCompositeState.Pickup }
                   .triggers {
                       intake.dynamicWrist(
                           max(intake.currentDynamicPosition() - 0.05, 0.0)
                       )
                   }
                   .repeatedlyWhilePressed()

               where(ButtonType.ButtonA)
                   .triggers {
                       if (intakeComposite.state == IntakeCompositeState.Rest)
                       {
                           intakeComposite.prepareForPickup()
                       } else
                       {
                           intakeComposite.cancelPickupAndReturnToRest()
                       }
                   }
                   .whenPressedOnce()
           }

            gp1Commands.doButtonUpdatesManually()
            gp2Commands.doButtonUpdatesManually()
        }

        fun motorTest()
        {
            gp1Commands.where(ButtonType.ButtonA)
                .triggers {
                    hardware.backLeft.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.backLeft.power = 0.0
                }

            gp1Commands.where(ButtonType.ButtonB)
                .triggers {
                    hardware.backRight.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.backRight.power = 0.0
                }

            gp1Commands.where(ButtonType.ButtonX)
                .triggers {
                    hardware.frontRight.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.frontRight.power = 0.0
                }

            gp1Commands.where(ButtonType.ButtonY)
                .triggers {
                    hardware.frontLeft.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    hardware.frontLeft.power = 0.0
                }
        }
    }

    override fun buildRobot() = TeleOpRobot(this)
}