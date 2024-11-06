package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig
import java.util.concurrent.CompletableFuture

class Extension(val robot: HypnoticRobot) : AbstractSubsystem()
{
    val slides = with(PIDFConfig(0.0025, 0.0, 0.0)) {
        ManagedMotorGroup(
            this@Extension,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            kF = { position, target, velocity ->
                val error = position - target
                if (error > 10 && error < 50) { // If extendo is close to being fully retracted, pull harder
                    -0.45
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