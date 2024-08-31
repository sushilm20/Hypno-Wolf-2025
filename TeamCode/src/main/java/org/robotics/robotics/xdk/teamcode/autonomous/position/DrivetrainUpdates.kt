package org.robotics.robotics.xdk.teamcode.autonomous.position

import org.robotics.robotics.xdk.teamcode.autonomous.AbstractAutoPipeline

data class DrivetrainUpdates(
    val newFrontLeft: Double,
    val newFrontRight: Double,
    val newBackLeft: Double,
    val newBackRight: Double
)
{
    fun propagate(pipeline: AbstractAutoPipeline)
    {
        pipeline.frontLeft.power = newFrontLeft
        pipeline.frontRight.power = newFrontRight
        pipeline.backLeft.power = newBackLeft
        pipeline.backRight.power = newBackRight
    }
}