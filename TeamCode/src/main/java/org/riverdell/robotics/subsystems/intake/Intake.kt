package org.riverdell.robotics.subsystems.intake

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.subsystems.motionProfiledServo
import org.riverdell.robotics.utilities.managed.ServoBehavior
import org.riverdell.robotics.utilities.motionprofile.Constraint
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

class Intake(private val robot: HypnoticRobot) : AbstractSubsystem() {
    val wrist = motionProfiledServo(robot.hardware.intakeWrist, ProfileConstraints(20.0, 2.0, 2.0))
    private val leftGrip =
        motionProfiledServo(robot.hardware.intakeClawLeft, Constraint.HALF.scale(10.5))
    private val rightGrip =
        motionProfiledServo(robot.hardware.intakeClawRight, Constraint.HALF.scale(10.5))

    var intakeState = IntakeState.Closed
    var wristState = WristState.Lateral

    private var dynamicPosition = 0.5
    fun currentDynamicPosition() = dynamicPosition

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
        if (wristState == state) {
            if (state != WristState.Dynamic) {
                return@let CompletableFuture.completedFuture(null)
            }
        }

        if (state == WristState.Dynamic) {
            wrist.unwrapServo().position = dynamicPosition
            return@let CompletableFuture.completedFuture(null)
        }

        dynamicPosition = state.position
        wristState = state
        return@let updateWristState()
    }

    private fun updateIntakeState(): CompletableFuture<*> {
        return intakeRotateTo(intakeState.position)
    }

    private fun updateWristState(): CompletableFuture<*> {
        if (wristState == WristState.Dynamic) {
            return wrist.setTarget(dynamicPosition, ServoBehavior.Direct)
        }

        return wristRotateTo(wristState.position)
    }

    private fun wristRotateTo(position: Double) = wrist.setTarget(position, ServoBehavior.Direct)
    private fun intakeRotateTo(position: Double) = CompletableFuture.allOf(
        leftGrip.setTarget(1.0 - position, ServoBehavior.Direct),
        rightGrip.setTarget(position, ServoBehavior.Direct)
    )

    override fun start() {

    }

    override fun doInitialize() {
        updateIntakeState()
        updateWristState()
    }
}