package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.Mono.commands
import io.liftgate.robotics.mono.gamepad.ButtonType
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

    }

    override fun opModeStart()
    {
        val robotDriver = GamepadEx(gamepad1)
        buildCommands()

        val thang = hardware<DcMotorEx>("elevator")
        while (opModeIsActive())
        {
            val multiplier = 0.5 + gamepad1.right_trigger * 0.5
            drivetrain.driveRobotCentric(robotDriver, multiplier)

            gp1Commands.run()
            gp2Commands.run()

            runPeriodics()

            multipleTelemetry.clearAll()
            multipleTelemetry.addLine(
                "Current position ${thang.currentPosition}"
            )
            multipleTelemetry.update()
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


        gp1Commands.doButtonUpdatesManually()
        gp2Commands.doButtonUpdatesManually()
    }
}