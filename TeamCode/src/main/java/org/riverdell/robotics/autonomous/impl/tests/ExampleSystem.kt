package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints

class ExampleSystem(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class CSClawConfig(
        var acceleration: Double = 8.0,
        var deceleration: Double = 8.0,
        var velocity: Double = 2.0,
        val leftIsReversed1Dot0Position: Boolean = false
    )

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
        leftRotation.setMotionProfileTarget(0.68)
            .thenCompose {
                leftRotation.setMotionProfileTarget(0.87)
            }
    }
}