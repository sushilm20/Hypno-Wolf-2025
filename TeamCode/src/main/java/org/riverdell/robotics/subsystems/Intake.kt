package org.riverdell.robotics.subsystems

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class Intake(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class WristConfig(
        var acceleration: Double = 0.0,
        var deceleration: Double = 0.0,
        var velocity: Double = 0.0
    )

    private val wristConfig = opMode.konfig<WristConfig>()
    private val wrist = ManagedServo(
        opMode.hardware("intake_wrist"),
        this@Intake
    ) {
        val config = wristConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    @Serializable
    data class GripConfig(
        var acceleration: Double = 0.0,
        var deceleration: Double = 0.0,
        var velocity: Double = 0.0,
        val leftIsReversed1Dot0Position: Boolean
    )

    private val gripConfig = opMode.konfig<GripConfig>()
    private val leftGrip = ManagedServo(
        opMode.hardware("intake_grip_left"),
        this@Intake
    ) {
        val config = gripConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    private val rightGrip = ManagedServo(
        opMode.hardware("intake_grip_right"),
        this@Intake
    ) {
        val config = gripConfig.get()
        ProfileConstraints(config.velocity, config.acceleration, config.deceleration)
    }

    fun wristRotateTo(position: Double) = wrist.setMotionProfileTarget(position)
    fun gripsRotateTo(position: Double) = CompletableFuture.allOf(
        leftGrip.setMotionProfileTarget(
            if (gripConfig.get().leftIsReversed1Dot0Position)
                (1.0 - position) else position
        ),
        rightGrip.setMotionProfileTarget(
            if (!gripConfig.get().leftIsReversed1Dot0Position)
                (1.0 - position) else position
        )
    )

    override fun doInitialize()
    {

    }
}