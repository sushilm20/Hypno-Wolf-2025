package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.movement.cubicBezier
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Lift(val robot: HypnoticRobot) : AbstractSubsystem()
{
    private val slides = with(PIDFConfig(0.01, 0.0, 0.0005)) {
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            kF = { position, targetPosition, velocity ->
                0.0
                /*val error = position - targetPosition
                if (error > 0 && error < 100) { // If elevator is just above the target position
                    if (error > 10) { // If elevator is more than 10 units above the target
                        -0.5// + velocity!! * 15 // Pull down hard at slow speed
                    } else {
                        0.0 // Don't pull down if very close to target
                    }
                } else {
                    if (error < 0)
                    {
                        0.5 // Resist gravity if below target position and going up
                    } else {
                        0.0 // Resist gravity less if going down
                    }
                }*/
            },
            master = robot.hardware.liftMotorLeft,
            slaves = listOf(robot.hardware.liftMotorRight)
        ).withTimeout(2500L)
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