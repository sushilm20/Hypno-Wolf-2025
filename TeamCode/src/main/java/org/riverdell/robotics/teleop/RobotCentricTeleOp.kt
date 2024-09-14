package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.Extension

/**
 * A TeleOp implementation which drives a mecanum drivebase
 * with robot-centric controls.
 *
 * @author Subham
 */
@TeleOp(name = "Multiplayer")
class RobotCentricTeleOp : AbstractTeleOp()
{
    override fun driveRobot(
        drivetrain: Drivetrain,
        driverOp: GamepadEx,
        multiplier: Double
    )
    {
        drivetrain.driveRobotCentric(driverOp, multiplier)
    }
}