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
import org.riverdell.robotics.utilities.managed.pidf.PIDFMotionProfiledConfig
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints

class ExampleSystem(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class CSClawConfig(
        var acceleration: Double = 8.0,
        var deceleration: Double = 2.0,
        var velocity: Double = 0.000025,
        val leftIsReversed1Dot0Position: Boolean = false
    )

    private val slidePIDFConfig = konfig<PIDFMotionProfiledConfig> { withCustomFileID("testpid_motionprofile") }

    val motorGroup = with(slidePIDFConfig.get()) {
        ManagedMotorGroup(
            this@ExampleSystem,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            master = opMode.hardware("elevator")
        )
            .configureMotors {
                direction = DcMotorSimple.Direction.FORWARD
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }
    }

    private val rotationConfig = konfig<CSClawConfig>()
    private val leftRotation = ManagedServo(
        opMode.hardware("extender"),
        this@ExampleSystem
    ) {
        val config = rotationConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    override fun start()
    {
        leftRotation.setMotionProfileTarget(0.5)

    }

    override fun doInitialize()
    {
        leftRotation.setMotionProfileTarget(0.87)
    }
}