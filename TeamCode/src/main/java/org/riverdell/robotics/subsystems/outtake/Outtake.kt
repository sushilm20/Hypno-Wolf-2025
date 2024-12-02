package org.riverdell.robotics.subsystems.outtake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.managed.ServoBehavior
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class Outtake(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    private val claw = motionProfiledServo(robot.hardware.outtakeClaw, Constraint.HALF.scale(10.5))

    private val coaxial = motionProfiledServo(robot.hardware.outtakeCoaxial, Constraint.HALF.scale(20.5))
    private val leftRotation = motionProfiledServo(robot.hardware.outtakeRotationLeft, Constraint.HALF.scale(20.5))
    private val rightRotation = motionProfiledServo(robot.hardware.outtakeRotationRight, Constraint.HALF.scale(20.5))

    var clawState = OuttakeClawState.Closed
    var coaxialState = OuttakeCoaxialState.Transfer
    var rotationState = OuttakeRotationState.Transfer

    fun transferRotation() = setRotation(OuttakeRotationState.Transfer)
    fun forceRotation() = setRotation(OuttakeRotationState.Force)
    fun depositRotation() = setRotation(OuttakeRotationState.Deposit)

    fun setRotation(state: OuttakeRotationState) = let {
        if (rotationState == state)
            return@let CompletableFuture.completedFuture(null)

        rotationState = state
        return@let updateRotationState()
    }

    private fun updateRotationState(): CompletableFuture<*>
    {
        return rotationRotateTo(rotationState.position)
    }

    fun openClaw() = setClaw(OuttakeClawState.Open)
    fun closeClaw() = setClaw(OuttakeClawState.Closed)

    fun setClaw(state: OuttakeClawState) = let {
        if (clawState == state)
            return@let CompletableFuture.completedFuture(null)

        clawState = state
        return@let updateClawState()
    }

    private fun updateClawState(): CompletableFuture<*>
    {
        return clawRotateTo(clawState.position)
    }

    fun transferCoaxial() = setCoaxial(OuttakeCoaxialState.Transfer)
    fun depositCoaxial() = setCoaxial(OuttakeCoaxialState.Deposit)
    fun outsideIntakeCoaxial() = setCoaxial(OuttakeCoaxialState.OutsideIntake)

    fun setCoaxial(state: OuttakeCoaxialState) = let {
        if (coaxialState == state)
            return@let CompletableFuture.completedFuture(null)

        coaxialState = state
        return@let updateCoaxialState()
    }

    private fun updateCoaxialState(): CompletableFuture<*>
    {
        return coaxialRotateTo(coaxialState.position)
    }

    private fun clawRotateTo(position: Double) = claw.setTarget(position, ServoBehavior.Direct)
    private fun coaxialRotateTo(position: Double) = coaxial.setTarget(position, ServoBehavior.MotionProfile)
    private fun rotationRotateTo(position: Double) = CompletableFuture.allOf(
        leftRotation.setTarget(position, ServoBehavior.MotionProfile),
        rightRotation.setTarget(position, ServoBehavior.MotionProfile)
    )

    override fun start()
    {

    }

    override fun doInitialize()
    {

    }
}