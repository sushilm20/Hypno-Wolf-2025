package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Extension(val robot: HypnoticRobot) : AbstractSubsystem()
{
    val slides = with(PIDFConfig(0.0025, 0.0, 0.0)) {
        ManagedMotorGroup(
            this@Extension,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            kF = { position, _, velocity ->
                if (position > 10 && position < 50 && velocity!! < 0) {
                    -0.3
                } else
                {
                    0.0
                }
            },
            master = robot.hardware.extensionMotorRight,
            slaves = listOf(robot.hardware.extensionMotorLeft)
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