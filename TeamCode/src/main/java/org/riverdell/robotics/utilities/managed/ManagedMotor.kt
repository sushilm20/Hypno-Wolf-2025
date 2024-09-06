package org.riverdell.robotics.utilities.managed

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.states.StateHolder
import kotlin.math.abs

class ManagedMotor(
    stateHolder: StateHolder,
    pid: PIDCoefficients,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kStatic: Double = 0.0,
    /**
     * Inputs: Measured position, measured velocity
     */
    private var kF: (Double, Double?) -> Double = { _, _ -> 0.0 },
    private val motor: DcMotorEx
)
{
    private var stuckProtection: StuckProtection? = null
    private var previousPosition: Int = 0

    private var stable = System.currentTimeMillis()

    private val pidfBuilder = {
        PIDFController(pid, kV, kA, kStatic, kF)
    }
    private var pidfController = pidfBuilder()

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
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER

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

    inner class FeedForwardDSL
    {

        fun static(value: Double)
        {
            kF = { _, _ -> value }
            pidfController = pidfBuilder()
        }

        fun none()
        {
            kF = { _, _ -> 0.0 }
            pidfController = pidfBuilder()
        }

        fun dynamic(block: (Double, Double?) -> Double)
        {
            kF = block
            pidfController = pidfBuilder()
        }

    }

    fun feedForward(block: FeedForwardDSL.() -> Unit) = FeedForwardDSL().block()

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