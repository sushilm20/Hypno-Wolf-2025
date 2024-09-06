package org.riverdell.robotics.autonomous.movement

import org.robotics.robotics.xdk.teamcode.autonomous.AutonomousWrapper

data class DrivetrainUpdates(
    val newFrontLeft: Double,
    val newFrontRight: Double,
    val newBackLeft: Double,
    val newBackRight: Double
)
{
    fun propagate(pipeline: AutonomousWrapper)
    {
        pipeline.frontLeft.power = newFrontLeft
        pipeline.frontRight.power = newFrontRight
        pipeline.backLeft.power = newBackLeft
        pipeline.backRight.power = newBackRight

        println(pipeline.frontLeft.power)
        println(pipeline.frontRight.power)
        println(pipeline.backLeft.power)
        println(pipeline.backRight.power)
    }
}