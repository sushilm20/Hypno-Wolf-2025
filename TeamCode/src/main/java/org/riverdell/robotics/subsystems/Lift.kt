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

class Lift(opMode: LinearOpMode) : AbstractSubsystem()
{
    private val leftSlide = opMode.hardware<DcMotorEx>("lift_motor_left")
        .apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
    private val rightSlide = opMode.hardware<DcMotorEx>("lift_motor_right")
        .apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

    private val slidePIDFConfig = opMode.konfig<LiftPIDF>()

    @Serializable
    data class LiftPIDF(
        var kP: Double = 0.0,
        var kI: Double = 0.0,
        var kD: Double = 0.0,
        var kV: Double = 0.0,
        var kA: Double = 0.0,
        var kStatic: Double = 0.0,
    )

    private val slides = with(slidePIDFConfig.get()) {
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = leftSlide,
            slaves = listOf(rightSlide)
        )
    }

    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun doInitialize()
    {
        slidePIDFConfig.onHotReload {
            slides.kA = kA
            slides.kV = kV
            slides.kStatic = kStatic
            slides.pid = PIDCoefficients(kP, kI, kD)
            slides.rebuild()
        }
    }

}