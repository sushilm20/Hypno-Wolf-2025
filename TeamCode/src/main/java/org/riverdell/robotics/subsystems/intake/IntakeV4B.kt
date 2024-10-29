package org.riverdell.robotics.subsystems.intake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class IntakeV4B(robot: HypnoticRobot) : AbstractSubsystem()
{
    private val leftRotation = motionProfiledServo("intakeV4BLeft", Constraint.HALF.scale(5.5))
    private val rightRotation = motionProfiledServo("intakeV4BRight", Constraint.HALF.scale(5.5))
    private val coaxialRotation = motionProfiledServo("intakeV4BCoaxial", Constraint.HALF.scale(4.5))

    var v4bState = V4BState.Lock
    var coaxialState = CoaxialState.Rest

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
    fun v4bUnlock() = setV4B(V4BState.UnlockedIdleHover)
    fun v4bSamplePickup() = setV4B(V4BState.Pickup)

    fun setV4B(state: V4BState) = let {
        if (state == v4bState)
        {
            return@let CompletableFuture.completedFuture(null)
        }

        v4bState = state
        return@let updateFourBarState()
    }

    private fun updateCoaxialState(): CompletableFuture<*>
    {
        return coaxialRotateTo(coaxialState.position)
    }

    private fun updateFourBarState(): CompletableFuture<Void>
    {
        return v4bRotateTo(v4bState.position)
    }

    private fun coaxialRotateTo(position: Double) = coaxialRotation.setMotionProfileTarget(position)
    private fun v4bRotateTo(position: Double) = CompletableFuture.allOf(
        leftRotation.setMotionProfileTarget(1.0 - position),
        rightRotation.setMotionProfileTarget(position)
    )

    override fun start()
    {
        updateCoaxialState()
            .exceptionally {
                it.printStackTrace()
                return@exceptionally null
            }
        updateFourBarState()
            .exceptionally {
                it.printStackTrace()
                return@exceptionally null
            }
    }

    override fun doInitialize()
    {

    }
}