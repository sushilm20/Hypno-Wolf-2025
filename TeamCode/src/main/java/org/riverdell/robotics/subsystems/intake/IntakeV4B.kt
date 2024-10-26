package org.riverdell.robotics.subsystems.intake

import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.states.StateResult
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import kotlinx.serialization.Serializable
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class IntakeV4B(robot: HypnoticRobot) : AbstractSubsystem()
{
    private val rotationConstraints = konfig<ProfileConstraints> { withCustomFileID("v4b_rotation_motionprofile") }
    private val leftRotation = motionProfiledServo("intakeV4BLeft", rotationConstraints)
    private val rightRotation = motionProfiledServo("intakeV4BRight", rotationConstraints)
    var v4bState = V4BState.Lock

    private val coaxialConstraints = konfig<ProfileConstraints> { withCustomFileID("v4b_coaxial_motionprofile") }
    private val coaxialRotation = motionProfiledServo("intakeV4BCoaxial", coaxialConstraints)
    var coaxialState = CoaxialState.Transfer

    fun coaxialIntermediate() = setCoaxial(CoaxialState.Intermediate)
    fun coaxialIntake() = setCoaxial(CoaxialState.Intake)
    fun coaxialTransfer() = setCoaxial(CoaxialState.Transfer)

    fun setCoaxial(state: CoaxialState) = let {
        if (state == coaxialState)
        {
            return@let CompletableFuture.completedFuture(null)
        }

        coaxialState = state
        return@let updateCoaxialState()
    }

    fun v4bLock() = setV4B(V4BState.Lock)
    fun v4bSampleSelect() = setV4B(V4BState.Select)
    fun v4bIntermediate() = setV4B(V4BState.Intermediate)
    fun v4bSamplePickup() = setV4B(V4BState.Pickup)

    fun setV4B(state: V4BState) = let {
        if (state == v4bState)
        {
            return@let CompletableFuture.completedFuture(null)
        }

        v4bState = state
        return@let updateFourBarState()
    }

    private fun updateCoaxialState(): CompletableFuture<Void>
    {
        return coaxialRotateTo(coaxialState.position)
    }

    private fun updateFourBarState(): CompletableFuture<Void>
    {
        return v4bRotateTo(v4bState.position)
    }

    private fun coaxialRotateTo(position: Double) = coaxialRotation.forcefullySetTarget(position)
    private fun v4bRotateTo(position: Double) = CompletableFuture.allOf(
        leftRotation.forcefullySetTarget(1.0 - position),
        rightRotation.forcefullySetTarget(position)
    )

    override fun start()
    {
        updateCoaxialState()
        updateFourBarState()
    }

    override fun doInitialize()
    {

    }
}