package org.riverdell.robotics

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class HypnoticOpMode : LinearOpMode()
{
    lateinit var robot: HypnoticRobot

    abstract fun buildRobot(): HypnoticRobot

    override fun runOpMode()
    {
        telemetry.speak("skibidi rizz is sigma sigma boy sigma boy sigma boy")
        telemetry.update()

        robot = buildRobot()
        robot.runOpMode()
    }
}