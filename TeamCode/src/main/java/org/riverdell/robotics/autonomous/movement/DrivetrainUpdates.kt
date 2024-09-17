package org.riverdell.robotics.autonomous.movement

import org.riverdell.robotics.autonomous.AutonomousWrapper

data class DrivetrainUpdates(
    val newFrontLeft: Double,
    val newFrontRight: Double,
    val newBackLeft: Double,
    val newBackRight: Double
)
{
    fun propagate(pipeline: AutonomousWrapper)
    {
        pipeline.drivetrain.frontLeft.power = newFrontLeft
        pipeline.drivetrain.frontRight.power = newFrontRight
        pipeline.drivetrain.backLeft.power = newBackLeft
        pipeline.drivetrain.backRight.power = newBackRight
    }
}