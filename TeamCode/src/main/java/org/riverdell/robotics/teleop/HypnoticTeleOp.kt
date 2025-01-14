package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.SampleType
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.intake.composite.IntakeConfig
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.outtake.PivotState
import java.util.concurrent.CompletableFuture
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

abstract class HypnoticTeleOp(internal val solo: Boolean = false) : HypnoticOpMode() {
    class TeleOpRobot(private val teleOp: HypnoticTeleOp) : HypnoticRobot(teleOp) {
        private val gp1Commands by lazy { commands(teleOp.gamepad1) }
        private val gp2Commands by lazy { commands(if (teleOp.solo) teleOp.gamepad1 else teleOp.gamepad2) }

//        val visionPipeline by lazy { VisionPipeline(teleOp) }

        override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands, /*visionPipeline*/)
        override fun initialize() {
//            visionPipeline.sampleDetection.supplyCurrentWristPosition { intake.wrist.unwrapServo().position }
//            visionPipeline.sampleDetection.setDetectionType(SampleType.BLUE)

            multipleTelemetry.addLine("Configured all subsystems. Waiting for start...")
            multipleTelemetry.update()

            while (!teleOp.isStarted) {
                runPeriodics()
            }
        }

        override fun opModeStart() {
            val robotDriver = GamepadEx(teleOp.gamepad1)
            buildCommands()

            multipleTelemetry.addLine("Started!")
            multipleTelemetry.update()

            intakeComposite.fromInitialToRest()

            var loopTime: Long
            while (teleOp.opModeIsActive()) {
                loopTime = System.nanoTime()
                runPeriodics()

                val multiplier =
                    0.5 + if (intakeComposite.state == InteractionCompositeState.Hover && teleOp.solo)
                        0.0 else (teleOp.gamepad1.right_trigger * 0.5)
                drivetrain.driveRobotCentric(robotDriver, multiplier)


                gp1Commands.run()
                gp2Commands.run()

                if (intakeComposite.state == InteractionCompositeState.InProgress) {
                    val timeInProgress = System.currentTimeMillis() - intakeComposite.attemptTime
                    if (timeInProgress > 2000L) {
                        if (intakeComposite.attemptedState != null) {
                            intakeComposite.state = intakeComposite.attemptedState!!
                            intakeComposite.attemptTime = System.currentTimeMillis()
                            intakeComposite.attemptedState = null
                        }
                    }
                }

                teleOp.telemetry.addData(
                    "Loop Refresh Rate ",
                    1000000000 / (System.nanoTime() - loopTime).toDouble()
                )
                teleOp.telemetry.addEssentialLines()
                teleOp.telemetry.update()
            }
        }

        private fun buildCommands() {
            gp1Commands.where(ButtonType.ButtonA)
                .triggers {
                    if (intakeComposite.state == InteractionCompositeState.DepositReady)
                    {
                        intakeComposite.scoreAndRest()
                        return@triggers
                    }

                    if (intakeComposite.state == InteractionCompositeState.Hover)
                    {
                        intakeComposite.fromHoverToDepositReady()
                        return@triggers
                    }

                    intakeComposite.fromRestToHover()
                }
                .whenPressedOnce()

            gp1Commands.where(ButtonType.BumperRight)
                .onlyWhen { intakeComposite.state == InteractionCompositeState.DepositReady }
                .triggers {
                    intakeComposite.outtakeNext()
                }
                .whenPressedOnce()

            gp1Commands.where(ButtonType.BumperLeft)
                .onlyWhen { intakeComposite.state == InteractionCompositeState.DepositReady }
                .triggers {
                    intakeComposite.outtakePrevious()
                }
                .whenPressedOnce()

            gp1Commands.where(ButtonType.BumperRight)
                .onlyWhen { intakeComposite.state == InteractionCompositeState.Hover }
                .triggers(50L) {
                    hardware.wrist.position = min(hardware.wrist.position + 0.02, 1.0)
                }
                .repeatedlyWhilePressed()

            gp1Commands.where(ButtonType.BumperLeft)
                .onlyWhen { intakeComposite.state == InteractionCompositeState.Hover }
                .triggers(50L) {
                    hardware.wrist.position = max(hardware.wrist.position - 0.02, 0.0)
                }
                .repeatedlyWhilePressed()

            gp1Commands.doButtonUpdatesManually()
            gp2Commands.doButtonUpdatesManually()
        }

        fun motorTest() {
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