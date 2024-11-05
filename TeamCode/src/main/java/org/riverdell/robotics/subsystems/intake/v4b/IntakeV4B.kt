package org.riverdell.robotics.subsystems.intake.v4b

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class IntakeV4B(robot: HypnoticRobot) : AbstractSubsystem()
{
    private val leftRotation = motionProfiledServo(robot.hardware.intakeV4BLeft, Constraint.HALF.scale(10.5))
    private val rightRotation = motionProfiledServo(robot.hardware.intakeV4BRight, Constraint.HALF.scale(10.5))
    private val coaxialRotation = motionProfiledServo(robot.hardware.intakeV4BCoaxial, Constraint.HALF.scale(10.5))

    var v4bState = V4BState.Lock
    var coaxialState = CoaxialState.Rest

    fun coaxialIntermediate() = setCoaxial(CoaxialState.Intermediate)
    fun coaxialIntake() = setCoaxial(CoaxialState.Intake)
    fun coaxialRest() = setCoaxial(CoaxialState.Rest)
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
    fun v4bSampleGateway() = setV4B(V4BState.Gateway)
//    fun v4bSampleFocus() = setV4B(V4BState.Focus)
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

    private fun updateFourBarState(): CompletableFuture<*>
    {
        return v4bRotateTo(v4bState.position)
    }

    private fun coaxialRotateTo(position: Double) = coaxialRotation.setMotionProfileTarget(position)
    private fun v4bRotateTo(position: Double): CompletableFuture<*>
    {
        rightRotation.setMotionProfileTarget(position)
        return leftRotation.setMotionProfileTarget(1.0 - position)
    }

    override fun start()
    {

    }

    override fun doInitialize()
    {
        updateCoaxialState()
        updateFourBarState()
    }
}