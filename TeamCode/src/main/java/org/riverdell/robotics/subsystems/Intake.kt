package org.riverdell.robotics.subsystems

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints
import java.util.concurrent.CompletableFuture

class Intake(opMode: LinearOpMode) : AbstractSubsystem()
{
    @Serializable
    data class IntakeConfig(val leftIsReversed: Boolean = false)

    private val intakeConfig = konfig<IntakeConfig>()

    private val wristConstraints = konfig<MotionProfileConstraints> { withCustomFileID("intake_wrist_motionprofile") }
    private val wrist = motionProfiledServo("intake_wrist", wristConstraints)

    private val rotationConstraints = konfig<MotionProfileConstraints> { withCustomFileID("intake_grip_motionprofile") }
    private val leftGrip = motionProfiledServo("intake_grip_left", rotationConstraints)
    private val rightGrip = motionProfiledServo("intake_grip_right", rotationConstraints)

    fun wristRotateTo(position: Double) = wrist.setMotionProfileTarget(position)
    fun gripsRotateTo(position: Double) = CompletableFuture.allOf(
        leftGrip.setMotionProfileTarget(
            if (intakeConfig.get().leftIsReversed)
                (1.0 - position) else position
        ),
        rightGrip.setMotionProfileTarget(
            if (!intakeConfig.get().leftIsReversed)
                (1.0 - position) else position
        )
    )

    override fun doInitialize()
    {

    }
}