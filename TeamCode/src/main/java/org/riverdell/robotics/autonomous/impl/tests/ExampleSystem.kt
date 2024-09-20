package org.riverdell.robotics.autonomous.impl.tests

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints
import org.riverdell.robotics.utilities.stopAndResetEncoder

class ExampleSystem(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class CSClawConfig(
        var acceleration: Double = 8.0,
        var deceleration: Double = 8.0,
        var velocity: Double = 2.0,
        val leftIsReversed1Dot0Position: Boolean = false
    )

    val motorGroup = ManagedMotorGroup(
        this,
        PIDCoefficients(5.0, 0.0, -0.2),
        master = opMode.hardware("elevator")
    ).configureMotors {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    private val rotationConfig = konfig<CSClawConfig>()
    private val leftRotation = ManagedServo(
        opMode.hardware("extender"),
        this@ExampleSystem
    ) {
        val config = rotationConfig.get()
        MotionProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    override fun doInitialize()
    {
        motorGroup.goTo(-300)
            .thenCompose {
                leftRotation.setMotionProfileTarget(0.87)
            }
            .thenCompose {
                leftRotation.setMotionProfileTarget(0.67)
            }
            .thenCompose {
                motorGroup.goTo(-500)
            }
    }
}