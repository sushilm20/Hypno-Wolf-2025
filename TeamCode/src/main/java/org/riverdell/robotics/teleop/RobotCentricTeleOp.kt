package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.riverdell.robotics.drivebase.Drivebase

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
        drivebase: Drivebase,
        driverOp: GamepadEx,
        multiplier: Double
    )
    {
        drivebase.driveRobotCentric(driverOp, multiplier)
    }
}