package org.riverdell.robotics.subsystems.outtake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.motionprofile.Constraint
import java.util.concurrent.CompletableFuture

class Outtake(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    private val claw = motionProfiledServo(robot.hardware.outtakeClaw, Constraint.HALF.scale(10.5))
//    private val leftRotation = motionProfiledServo(robot.hardware.outtakeRotationLeft, Constraint.HALF.scale(10.5))
    private val rightRotation = motionProfiledServo(robot.hardware.outtakeRotationRight, Constraint.HALF.scale(10.5))

    var clawState = OuttakeClawState.Closed
    var rotationState = OuttakeRotationState.Transfer

    fun transferRotation() = setRotation(OuttakeRotationState.Transfer)
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

    private fun clawRotateTo(position: Double) = claw.forcefullySetTarget(position)
    private fun rotationRotateTo(position: Double) = rightRotation.forcefullySetTarget(position)/*CompletableFuture.allOf(
//        leftRotation.forcefullySetTarget(position),
        rightRotation.forcefullySetTarget(position)
    )*/

    override fun start()
    {

    }

    override fun doInitialize()
    {
        updateRotationState()
        updateClawState()
    }
}