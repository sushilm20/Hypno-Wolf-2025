package org.riverdell.robotics.subsystems.intake

import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.states.StateResult
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.motionprofile.Constraint
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class Intake(opMode: HypnoticRobot) : AbstractSubsystem()
{
    private val wrist = motionProfiledServo("intakeWrist", Constraint.HALF.scale(6.5))
    private val leftGrip = motionProfiledServo("intakeClawLeft", Constraint.HALF.scale(10.5))
    private val rightGrip = motionProfiledServo("intakeClawRight", Constraint.HALF.scale(10.5))

    var intakeState = IntakeState.Closed
    var wristState = WristState.Lateral

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

    private fun updateWristState(): CompletableFuture<*>
    {
        if (wristState == WristState.Dynamic)
        {
            return wristRotateTo(dynamicPosition)
        }

        return wristRotateTo(wristState.position)
    }

    private fun wristRotateTo(position: Double) = wrist.setMotionProfileTarget(position)
    private fun intakeRotateTo(position: Double) = CompletableFuture.allOf(
        leftGrip.forcefullySetTarget(1.0 - position),
        rightGrip.forcefullySetTarget(position)
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