package org.riverdell.robotics.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig
import org.riverdell.robotics.utilities.managed.pidf.PIDFMotionProfiledConfig

class Extension(opMode: LinearOpMode) : AbstractSubsystem()
{
    private val leftSlide = opMode.hardware<DcMotorEx>("extension_motor_left")
        .apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
    private val rightSlide = opMode.hardware<DcMotorEx>("extension_motor_right")
        .apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

    private val slidePIDFConfig = konfig<PIDFMotionProfiledConfig> { withCustomFileID("extension") }
    private val slides = with(slidePIDFConfig.get()) {
        ManagedMotorGroup(
            this@Extension,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = leftSlide,
            slaves = listOf(rightSlide)
        ).withMotionProfiling(motionProfileConstraints)
    }

    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun doInitialize()
    {

    }

}