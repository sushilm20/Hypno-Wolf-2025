package org.riverdell.robotics.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFMotionProfiledConfig

class Lift(opMode: HypnoticOpMode) : AbstractSubsystem()
{
    private val slidePIDFConfig = konfig<PIDFMotionProfiledConfig> { withCustomFileID("lift") }
    private val slides = with(slidePIDFConfig.get()) {
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = opMode.robot.hardware.liftMotorLeft,
            slaves = listOf(opMode.robot.hardware.liftMotorRight)
        )
    }

    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun start()
    {

    }

    override fun doInitialize()
    {
    }

}