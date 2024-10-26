package org.riverdell.robotics.subsystems.intake

import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.states.StateResult
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class Intake(opMode: HypnoticRobot) : AbstractSubsystem()
{
    @Serializable
    data class IntakeConfig(val leftIsReversed: Boolean = true)

    private val intakeConfig = konfig<IntakeConfig>()

    private val wristConstraints = konfig<ProfileConstraints> { withCustomFileID("intake_wrist_motionprofile") }
    private val wrist = motionProfiledServo("intakeWrist", wristConstraints)

    private val rotationConstraints = konfig<ProfileConstraints> { withCustomFileID("intake_grip_motionprofile") }
    private val leftGrip = motionProfiledServo("intakeClawLeft", rotationConstraints)
    private val rightGrip = motionProfiledServo("intakeClawRight", rotationConstraints)

    private var intakeState = IntakeState.Closed
    private var wristState = WristState.Lateral

    private var dynamicPosition = 0.5

    fun openIntake() = setIntake(IntakeState.Open)
    fun closeIntake() = setIntake(IntakeState.Closed)

    fun setIntake(state: IntakeState) = let {
        if (intakeState == state)
            return@let CompletableFuture.completedFuture(null)

        intakeState = state
        return@let updateIntakeState()
    }

    fun lateralWrist() = setWrist(WristState.Lateral)
    fun perpendicularWrist() = setWrist(WristState.Perpendicular)
    fun dynamicWrist(position: Double) = let {
        dynamicPosition = position
        return@let setWrist(WristState.Dynamic)
    }

    fun setWrist(state: WristState) = let {
        if (wristState == state && wristState != WristState.Dynamic)
            return@let CompletableFuture.completedFuture(null)

        wristState = state
        return@let updateWristState()
    }

    private fun updateIntakeState(): CompletableFuture<Void>
    {
        return intakeRotateTo(intakeState.position)
    }

    private fun updateWristState(): CompletableFuture<StateResult>
    {
        if (wristState == WristState.Dynamic)
        {
            return wristRotateTo(dynamicPosition)
        }

        return wristRotateTo(wristState.position)
    }

    private fun wristRotateTo(position: Double) = wrist.setMotionProfileTarget(position)
    private fun intakeRotateTo(position: Double) = CompletableFuture.allOf(
        leftGrip.setMotionProfileTarget(
            if (intakeConfig.get().leftIsReversed)
                (1.0 - position) else position
        ),
        rightGrip.setMotionProfileTarget(
            if (!intakeConfig.get().leftIsReversed)
                (1.0 - position) else position
        )
    )

    override fun start()
    {
        updateIntakeState()
        updateWristState()
    }

    override fun doInitialize()
    {

    }
}