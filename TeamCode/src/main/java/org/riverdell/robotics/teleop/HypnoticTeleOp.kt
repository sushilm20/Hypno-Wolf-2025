package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.intake.composite.IntakeConfig
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

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
                runPeriodics()

                val multiplier = 0.5 + opMode.gamepad1.right_trigger * 0.5
                drivetrain.driveRobotCentric(robotDriver, multiplier)

                gp1Commands.run()
                gp2Commands.run()

                if (intakeComposite.state == InteractionCompositeState.Pickup)
                {
                    val wantedPower = -opMode.gamepad2.left_trigger + opMode.gamepad2.right_trigger
                    if (wantedPower.absoluteValue > 0.1/* && !extension.slides.isTravelling()*/)
                    {
                        if (wantedPower < 0)
                        {
                            if (extension.slides.currentPosition() < 35)
                            {
                                extension.slides.supplyPowerToAll(0.0)
                            } else
                            {
                                extension.slides.supplyPowerToAll((wantedPower.toDouble().pow(2) * sign(wantedPower.toDouble())) / 6)
                            }
                        } else
                        {
                            if (extension.slides.currentPosition() >= IntakeConfig.MAX_EXTENSION)
                            {
                                extension.slides.supplyPowerToAll(0.0)
                            } else
                            {
                                extension.slides.supplyPowerToAll((wantedPower.toDouble().pow(2) * sign(wantedPower.toDouble())) / 6)
                            }
                        }
                    } else if (!extension.slides.isTravelling())
                    {
                        extension.slides.supplyPowerToAll(0.0)
                    }
                }

                if (intakeComposite.state == InteractionCompositeState.InProgress)
                {
                    val timeInProgress = System.currentTimeMillis() - intakeComposite.attemptTime
                    if (timeInProgress > 2000L)
                    {
                        if (intakeComposite.attemptedState != null)
                        {
                            intakeComposite.state = intakeComposite.attemptedState!!
                            intakeComposite.attemptTime = System.currentTimeMillis()
                            intakeComposite.attemptedState = null
                        }
                    }
                }

                opMode.telemetry.addData("Loop Refresh Rate ", 1000000000 / (System.nanoTime() - loopTime).toDouble())
                opMode.telemetry.addEssentialLines()
                opMode.telemetry.update()
            }
        }

        private fun buildCommands()
        {
           gp2Commands.apply {
               where(ButtonType.DPadLeft)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                   .triggers {
                       intake.perpendicularWrist()
                   }
                   .whenPressedOnce()

               where(ButtonType.DPadRight)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                   .triggers {
                       intake.lateralWrist()
                   }
                   .whenPressedOnce()

               where(ButtonType.BumperRight)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                   .triggers {
                       intake.dynamicWrist(
                           min(intake.currentDynamicPosition() + 0.02, 1.0)
                       )
                   }
                   .repeatedlyWhilePressed()

               where(ButtonType.BumperLeft)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Pickup }
                   .triggers {
                       intake.dynamicWrist(
                           max(intake.currentDynamicPosition() - 0.02, 0.0)
                       )
                   }
                   .repeatedlyWhilePressed()

               where(ButtonType.DPadLeft)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.OuttakeReady }
                   .triggers {
                       intakeComposite.exitOuttakeReadyToRest()
                   }
                   .whenPressedOnce()

               where(ButtonType.DPadRight)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Outtaking }
                   .triggers {
                       intakeComposite.outtakeCompleteAndReturnToOuttakeReady()
                   }
                   .whenPressedOnce()

               where(ButtonType.DPadUp)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Outtaking }
                   .triggers {
                       intakeComposite.outtakeNext()
                   }
                   .whenPressedOnce()

               where(ButtonType.DPadDown)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.Outtaking }
                   .triggers {
                       intakeComposite.outtakePrevious()
                   }
                   .whenPressedOnce()

               where(ButtonType.DPadUp)
                   .onlyWhen { intakeComposite.state == InteractionCompositeState.OuttakeReady }
                   .triggers {
                       intakeComposite.initialOuttake()
                   }
                   .whenPressedOnce()

               where(ButtonType.ButtonY)
                   .onlyWhen {
                       intakeComposite.state != InteractionCompositeState.InProgress
                   }
                   .triggers {
                       if (intakeComposite.state == InteractionCompositeState.Rest)
                       {
                           intakeComposite.wallOuttakeFromRest()
                       } else if (intakeComposite.state == InteractionCompositeState.WallIntakeViaOuttake)
                       {
                           intakeComposite.wallOuttakeToOuttakeReady()
                       }
                   }
                   .whenPressedOnce()

               where(ButtonType.ButtonX)
                   .onlyWhen {
                       intakeComposite.state != InteractionCompositeState.InProgress &&
                           intakeComposite.state == InteractionCompositeState.Confirm
                   }
                   .triggers {
                       intakeComposite.declineAndIntake()
                   }
                   .whenPressedOnce()

               where(ButtonType.ButtonA)
                   .onlyWhen {
                       intakeComposite.state != InteractionCompositeState.InProgress
                   }
                   .triggers {
                       if (intakeComposite.state == InteractionCompositeState.Rest)
                       {
                           intakeComposite.prepareForPickup()
                       } else
                       {
                           if (intakeComposite.state == InteractionCompositeState.Pickup)
                           {
                               intakeComposite.intakeAndConfirm()
                           } else if (intakeComposite.state == InteractionCompositeState.Confirm)
                           {
                               intakeComposite.confirmAndTransferAndReady()
                           }
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