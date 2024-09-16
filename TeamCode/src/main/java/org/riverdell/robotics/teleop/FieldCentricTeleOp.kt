package org.riverdell.robotics.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.Extension

/**
 * A TeleOp implementation which drives a mecanum drivebase
 * with field-centric controls using the built-in IMU.
 *
 * @author Subham
 */
@Disabled
@TeleOp(name = "Multiplayer (Field Centric)")
class FieldCentricTeleOp : AbstractTeleOp()
{

    override fun driveRobot(drivetrain: Drivetrain, driverOp: GamepadEx, multiplier: Double)
    {
        drivetrain.driveFieldCentric(driverOp, multiplier)
    }
}