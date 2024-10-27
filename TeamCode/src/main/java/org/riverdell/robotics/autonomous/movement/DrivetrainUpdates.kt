package org.riverdell.robotics.autonomous.movement

import org.riverdell.robotics.autonomous.HypnoticAuto

data class DrivetrainUpdates(
    val newFrontLeft: Double,
    val newFrontRight: Double,
    val newBackLeft: Double,
    val newBackRight: Double
)
{
    fun propagate(pipeline: HypnoticAuto)
    {
        pipeline.robot.drivetrain.frontLeft.power = newFrontLeft
        pipeline.robot.drivetrain.frontRight.power = newFrontRight
        pipeline.robot.drivetrain.backLeft.power = newBackLeft
        pipeline.robot.drivetrain.backRight.power = newBackRight
    }
}