package org.riverdell.robotics.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Extension(robot: HypnoticRobot) : AbstractSubsystem()
{
    val leftSlide = robot.hardware<DcMotorEx>("extendoLeft")
        .apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
    val rightSlide = robot.hardware<DcMotorEx>("extendoRight")
        .apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

    private val slides = with(PIDFConfig(0.0025, 0.0, 0.0)) {
        ManagedMotorGroup(
            this@Extension,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = rightSlide,
            slaves = listOf(leftSlide)
        ).withTimeout(1500L)
    }

    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    var thing = 0

    override fun start()
    {
        println("Extension number ${thing + 1}")
//        extendToAndStayAt(0)
    }

    override fun doInitialize()
    {

    }

}