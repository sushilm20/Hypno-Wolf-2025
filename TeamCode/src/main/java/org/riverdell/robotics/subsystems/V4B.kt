package org.riverdell.robotics.subsystems

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class V4B(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class V4BRotationConfig(
        var acceleration: Double = 0.0,
        var deceleration: Double = 0.0,
        var velocity: Double = 0.0,
        val leftIsReversed1Dot0Position: Boolean
    )

    private val rotationConfig = opMode.konfig<V4BRotationConfig>()
    private val leftRotation = ManagedServo(
        opMode.hardware("v4b_rotation_left"),
        this@V4B
    ) {
        val config = rotationConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    private val rightRotation = ManagedServo(
        opMode.hardware("v4b_rotation_right"),
        this@V4B
    ) {
        val config = rotationConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    @Serializable
    data class V4BCoaxialRotationConstraints(
        var acceleration: Double = 0.0,
        var deceleration: Double = 0.0,
        var velocity: Double = 0.0
    )

    private val rotationCoaxialConstraints = opMode.konfig<V4BCoaxialRotationConstraints>()
    private val coaxialRotation = ManagedServo(
        opMode.hardware("v4b_coaxial"),
        this@V4B
    ) {
        val config = rotationCoaxialConstraints.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    fun coaxialRotateTo(position: Double) = coaxialRotation.setMotionProfileTarget(position)
    fun rotateTo(position: Double) = CompletableFuture.allOf(
        leftRotation.setMotionProfileTarget(
            if (rotationConfig.get().leftIsReversed1Dot0Position)
                (1.0 - position) else position
        ),
        rightRotation.setMotionProfileTarget(
            if (!rotationConfig.get().leftIsReversed1Dot0Position)
                (1.0 - position) else position
        )
    )

    override fun doInitialize()
    {

    }
}