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
        pipeline.robot.hardware.frontLeft.power = newFrontLeft
        pipeline.robot.hardware.frontRight.power = newFrontRight
        pipeline.robot.hardware.backLeft.power = newBackLeft
        pipeline.robot.hardware.backRight.power = newBackRight
    }
}