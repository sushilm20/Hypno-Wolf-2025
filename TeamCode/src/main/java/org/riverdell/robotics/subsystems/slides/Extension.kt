package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Extension(val robot: HypnoticRobot) : AbstractSubsystem()
{
    val slides = with(PIDFConfig(0.004, 0.0, 0.0)) {
        ManagedMotorGroup(
            this@Extension,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = robot.hardware.extensionMotorLeft,
            slaves = listOf(robot.hardware.extensionMotorRight)
        ).withTimeout(2000L)
    }

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