package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
import io.liftgate.robotics.mono.gamepad.bundle
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.hardware

@TeleOp(
    name = "Multiplayer",
    group = "Drive"
)
class HypnoticTeleOp : HypnoticRobot()
{
    private val gp1Commands by lazy { commands(gamepad1) }
    private val gp2Commands by lazy { commands(gamepad2) }

    override fun additionalSubSystems() = listOf(gp1Commands, gp2Commands)
    override fun initialize()
    {
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
        /*gp1Commands.where(ButtonType.ButtonA)
            .triggers {
                test.motorGroup.goTo(-500)

            }.andIsHeldUntilReleasedWhere {
                test.motorGroup.goTo(-100)
            }*/

        val defaultBundleDriver1 = gamepad1.bundle {
            where(ButtonType.ButtonA)
                .triggers {
                    println("hello")
                }
                .whenPressedOnce()
        }

        val defaultBundleDriver2 = gamepad1.bundle {
            where(ButtonType.ButtonB)
                .triggers {
                    println("NOW EXTENDO OUT")
                }
                .whenPressedOnce()

            where(ButtonType.ButtonY)
                .triggers {
                    println("RETURNING BACK")
                    defaultBundleDriver1.applyTo(gp1Commands)
                }
                .whenPressedOnce()
        }

        gp1Commands
            .where(ButtonType.ButtonX)
            .triggers {
                defaultBundleDriver2.applyTo(gp1Commands)
            }
            .whenPressedOnce()

        gp1Commands.doButtonUpdatesManually()
        gp2Commands.doButtonUpdatesManually()
    }
}