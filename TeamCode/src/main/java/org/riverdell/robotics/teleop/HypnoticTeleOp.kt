package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import io.liftgate.robotics.mono.gamepad.bundle
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.SampleType
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.subsystems.intake.V4BState
import org.riverdell.robotics.utilities.hardware

@TeleOp(
    name = "Multiplayer",
    group = "Drive"
)
class HypnoticTeleOp : HypnoticRobot()
{
    private val gp1Commands by lazy { commands(gamepad1) }
    private val gp2Commands by lazy { commands(gamepad2) }

    val visionPipeline by lazy { VisionPipeline(this) }

    override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands, /*visionPipeline*/)
    override fun initialize()
    {
        /*wrist.position = 0.5
        visionPipeline.sampleDetection.supplyCurrentWristPosition { wrist.position }
        visionPipeline.sampleDetection.setDetectionType(SampleType.Blue)*/

        while (!isStarted)
        {
            multipleTelemetry.addLine("Configured all subsystems. Waiting for start...")
            multipleTelemetry.update()
            runPeriodics()
        }
    }

    override fun opModeStart()
    {
        val robotDriver = GamepadEx(gamepad1)
        buildCommands()
        while (opModeIsActive())
        {
            val multiplier = 0.5 + gamepad1.right_trigger * 0.5
            drivetrain.driveRobotCentric(robotDriver, multiplier)

            gp1Commands.run()
            gp2Commands.run()

            runPeriodics()
        }
    }

    private fun buildCommands()
    {
        gp1Commands
            .where(ButtonType.PlayStationCircle)
            .triggers {
                intake.perpendicularWrist()
            }
            .whenPressedOnce()

        gp1Commands
            .where(ButtonType.ButtonY)
            .triggers {
                if (intakeV4B.v4bState == V4BState.Lock)
                {
                    intakeV4B.coaxialIntake()
                    intakeV4B.v4bSampleSelect()
                        .thenCompose {
                            intake.openIntake()
                        }
                } else
                {
                    intake.closeIntake()
                        .thenCompose {
                            intake.lateralWrist()
                        }
                        .thenAccept {
                            intakeV4B.coaxialTransfer()
                            intakeV4B.v4bLock()
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