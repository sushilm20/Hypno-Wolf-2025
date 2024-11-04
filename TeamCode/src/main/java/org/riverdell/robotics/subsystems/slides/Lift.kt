package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.movement.cubicBezier
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Lift(val robot: HypnoticRobot) : AbstractSubsystem()
{
    private val slides = with(PIDFConfig(0.005, 0.0, 0.0)) {
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            kF = { position, targetPosition, velocity ->
                if ((position - targetPosition) < 100)
                {
                    if ((position - targetPosition) > 10)
                    {
                        -0.4
                    } else {
                        if (position < 10)
                        {
                            0.0
                        } else {
                            0.35
                        }
                    }
                } else
                {
                    0.0
                }
            },
            master = robot.hardware.liftMotorRight,
            slaves = listOf(robot.hardware.liftMotorLeft)
        ).withTimeout(1500L)
    }

    fun position() = robot.hardware.liftMotorRight.currentPosition
    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun start()
    {
//        extendToAndStayAt(0)
    }

    override fun doInitialize()
    {

    }

}