package org.robotics.robotics.xdk.teamcode.subsystem.pidf

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.states.StateHolder
import kotlin.math.abs

class StatePIDFMotor(
    stateHolder: StateHolder,
    pid: PIDCoefficients,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kStatic: Double = 0.0,
    kF: (Double, Double?) -> Double = { _, _ -> 0.0 },
    private val motor: DcMotorEx
)
{
    private var stuckProtection: StuckProtection? = null
    private var previousPosition: Int = 0

    private var stable = System.currentTimeMillis()
    private val pidfController = PIDFController(pid, kV, kA, kStatic, kF)

    private val state by stateHolder.state<Int>(
        write = {
            stable = System.currentTimeMillis()
            pidfController.targetPosition = it.toDouble()
        },
        read = {
            motor.currentPosition
        },
        complete = { current, target ->
            current == target || if (stuckProtection == null)
            {
                false
            } else
            {
                val diffs = abs(current - previousPosition)
                previousPosition = current

                if (diffs > stuckProtection!!.minimumRequiredPositionDifference)
                {
                    stable = System.currentTimeMillis()
                    false
                } else
                {
                    System.currentTimeMillis() - stable > stuckProtection!!.timeStuckUnderMinimumMillis
                }
            }
        }
    )

    init
    {
        /**
         * Continuously update the power of the motor to keep it at the
         * desired target value set in the [PIDFController].
         */
        state.additionalPeriodic { current, _ ->
            val velocity = motor.velocity
            motor.power = pidfController
                .update(
                    measuredPosition = current.toDouble(),
                    measuredVelocity = velocity
                )
        }
    }

    fun enableStuckProtection(stuckProtection: StuckProtection) = apply {
        this.stuckProtection = stuckProtection
    }

    fun configure(block: PIDFController.() -> Unit) = apply {
        pidfController.block()
    }

    fun configureMotor(block: DcMotorEx.() -> Unit) = apply {
        motor.block()
    }

    fun isTravelling() = state.inProgress()
    fun goTo(target: Int) = state.override(target)
    fun reset() = state.reset()
}