package org.riverdell.robotics.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Lift(val robot: HypnoticRobot) : AbstractSubsystem()
{
    private val slides = with(PIDFConfig(0.0025, 0.0, 0.0)) {
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = robot.hardware.extensionMotorRight,
            slaves = listOf(robot.hardware.extensionMotorLeft)
        ).withTimeout(1500L)
    }

    fun position() = robot.hardware.liftMotorRight.currentPosition
    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun start()
    {
        extendToAndStayAt(0)
    }

    override fun doInitialize()
    {

    }

}