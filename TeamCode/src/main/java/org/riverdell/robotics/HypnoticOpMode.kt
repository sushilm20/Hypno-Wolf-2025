package org.riverdell.robotics

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class HypnoticOpMode : LinearOpMode()
{
    lateinit var robot: HypnoticRobot

    abstract fun buildRobot(): HypnoticRobot

    override fun runOpMode()
    {
        robot = buildRobot()
        robot.runOpMode()
    }
}