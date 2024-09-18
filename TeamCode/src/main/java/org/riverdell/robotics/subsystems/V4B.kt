package org.riverdell.robotics.subsystems

import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints
import java.util.concurrent.CompletableFuture

class V4B(opMode: HypnoticRobot) : AbstractSubsystem()
{
    @Serializable
    data class V4BConfig(val leftIsReversed: Boolean = false)

    private val v4bConfig = konfig<V4BConfig>()

    private val rotationConstraints = konfig<MotionProfileConstraints> { withCustomFileID("v4b_rotation_motionprofile") }
    private val leftRotation = motionProfiledServo("v4b_rotation_left", rotationConstraints)
    private val rightRotation = motionProfiledServo("v4b_rotation_right", rotationConstraints)

    private val coaxialConstraints = konfig<MotionProfileConstraints>()
    private val coaxialRotation = motionProfiledServo("v4b_coaxial", coaxialConstraints)

    fun coaxialRotateTo(position: Double) = coaxialRotation.setMotionProfileTarget(position)
    fun rotateTo(position: Double) = CompletableFuture.allOf(
        leftRotation.setMotionProfileTarget(
            if (v4bConfig.get().leftIsReversed)
                (1.0 - position) else position
        ),
        rightRotation.setMotionProfileTarget(
            if (!v4bConfig.get().leftIsReversed)
                (1.0 - position) else position
        )
    )

    override fun doInitialize()
    {

    }
}