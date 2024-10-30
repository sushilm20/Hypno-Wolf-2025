package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import io.liftgate.robotics.mono.gamepad.CommandBundle
import io.liftgate.robotics.mono.gamepad.GamepadCommands
import io.liftgate.robotics.mono.gamepad.bundle
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.subsystems.intake.composite.IntakeCompositeState
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
            while (opMode.opModeIsActive())
            {
                val multiplier = 0.5 + opMode.gamepad1.right_trigger * 0.5
                drivetrain.driveRobotCentric(robotDriver, multiplier)

                opMode.telemetry.addLine("Intake State: ${intake.intakeState}")
                opMode.telemetry.addLine("Wrist State: ${intake.wristState}")
                opMode.telemetry.addLine("4BR State: ${intakeV4B.v4bState}")
                opMode.telemetry.addLine("Coaxial State: ${intakeV4B.coaxialState}")

                opMode.telemetry.addLine("Extendo Right (MASTER) Position: ${hardware.extensionMotorRight.currentPosition}")
                opMode.telemetry.update()

                gp1Commands.run()
                gp2Commands.run()

                runPeriodics()
            }
        }

        lateinit var intakeCommands: CommandBundle
        lateinit var defaultCommands: CommandBundle

        private fun buildCommands()
        {
            intakeCommands = opMode.gamepad2.bundle {
                where(ButtonType.DPadLeft)
                    .triggers {
                        intake.perpendicularWrist()
                    }
                    .whenPressedOnce()

                where(ButtonType.DPadRight)
                    .triggers {
                        intake.lateralWrist()
                    }
                    .whenPressedOnce()

                where(ButtonType.BumperRight)
                    .triggers(100L) {
                        intake.dynamicWrist(
                            min(intake.currentDynamicPosition() + 0.05, 1.0)
                        )
                    }
                    .repeatedlyWhilePressed()

                where(ButtonType.BumperLeft)
                    .triggers(100L) {
                        intake.dynamicWrist(
                            max(intake.currentDynamicPosition() - 0.05, 0.0)
                        )
                    }
                    .repeatedlyWhilePressed()

                where(ButtonType.ButtonA)
                    .triggers {
                        intakeComposite.cancelPickupAndReturnToRest() // TODO: Transfer
                        defaultCommands.applyTo(gp2Commands)
                    }
                    .whenPressedOnce()
            }

            defaultCommands = opMode.gamepad2.bundle {
                where(ButtonType.ButtonA)
                    .triggers {
                        intakeComposite.prepareForPickup()
                        intakeCommands.applyTo(gp2Commands)
                    }
                    .whenPressedOnce()
            }

            defaultCommands.applyTo(gp2Commands)

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