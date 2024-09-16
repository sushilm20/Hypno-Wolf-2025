package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class ExampleSubSystem(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class V4BRotationConfig(
        var acceleration: Double = 8.0,
        var deceleration: Double = 8.0,
        var velocity: Double = 2.0,
        val leftIsReversed1Dot0Position: Boolean = false
    )

    // 0.68
    //0.87

    private val rotationConfig = opMode.konfig<V4BRotationConfig>()
    private val leftRotation = ManagedServo(
        opMode.hardware("extender"),
        this@ExampleSubSystem
    ) {
        val config = rotationConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    override fun doInitialize()
    {
        leftRotation.setMotionProfileTarget(0.68)
            .thenRun {
                println("HEYY")
            }
    }
}