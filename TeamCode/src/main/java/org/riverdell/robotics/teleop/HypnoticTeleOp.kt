package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.subsystems.intake.V4BState
import org.riverdell.robotics.utilities.hardware
import java.util.concurrent.CompletableFuture
import kotlin.concurrent.thread

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

                opMode.telemetry.addLine("Extendo Left (MASTER) Position: ${extension.leftSlide.currentPosition}")
                opMode.telemetry.addLine("Extendo Right (MASTER) Position: ${extension.rightSlide.currentPosition}")

                opMode.telemetry.addLine(
                    "Extendo Right Current: ${
                        extension.rightSlide.getCurrent(
                            CurrentUnit.AMPS
                        )
                    }"
                )
                opMode.telemetry.addLine(
                    "Extendo Left (MASTER) Current: ${
                        extension.rightSlide.getCurrent(
                            CurrentUnit.AMPS
                        )
                    }"
                )
                opMode.telemetry.update()

                gp1Commands.run()
                gp2Commands.run()

                runPeriodics()
            }
        }

        private fun buildCommands()
        {
            gp1Commands
                .where(ButtonType.BumperLeft)
                .triggers {
                    intake.perpendicularWrist()
                }
                .whenPressedOnce()

            gp1Commands
                .where(ButtonType.BumperRight)
                .triggers {
                    intake.lateralWrist()
                }
                .whenPressedOnce()

            gp1Commands
                .where(ButtonType.PlayStationCircle)
                .triggers {
                    intake.openIntake()
                }
                .whenPressedOnce()

            gp1Commands
                .where(ButtonType.ButtonY)
                .triggers {
                    if (intakeV4B.v4bState == V4BState.Lock)
                    {
                        intakeV4B.v4bUnlock()
                            .thenCompose {
                                CompletableFuture.allOf(
                                    extension.extendToAndStayAt(424),
                                    intakeV4B.v4bSamplePickup(),
                                    intakeV4B.coaxialIntake(),
                                    intake.lateralWrist()
                                )
                            }

                        intake.openIntake()
                    } else
                    {
                        intakeV4B.v4bSamplePickup()
                            .thenCompose {
                                CompletableFuture.allOf(
                                    intake.closeIntake(),
                                    intakeV4B.coaxialTransfer(),
                                    intake.lateralWrist(),
                                    intakeV4B.v4bLock(),
                                    extension.extendToAndStayAt(0)
                                )
                            }
                    }
                }
                .whenPressedOnce()

            gp1Commands.doButtonUpdatesManually()
            gp2Commands.doButtonUpdatesManually()
        }

        fun motorTest()
        {
            val backLeft = hardware<DcMotorEx>("backLeft")
            gp1Commands.where(ButtonType.ButtonA)
                .triggers {
                    backLeft.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    backLeft.power = 0.0
                }

            val backRight = hardware<DcMotorEx>("backRight")
            gp1Commands.where(ButtonType.ButtonB)
                .triggers {
                    backRight.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    backRight.power = 0.0
                }

            val frontRight = hardware<DcMotorEx>("frontRight")
            gp1Commands.where(ButtonType.ButtonX)
                .triggers {
                    frontRight.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    frontRight.power = 0.0
                }

            val frontLeft = hardware<DcMotorEx>("frontLeft")
            gp1Commands.where(ButtonType.ButtonY)
                .triggers {
                    frontLeft.power = 1.0
                }
                .andIsHeldUntilReleasedWhere {
                    frontLeft.power = 0.0
                }
        }
    }

    override fun buildRobot() = TeleOpRobot(this)
}