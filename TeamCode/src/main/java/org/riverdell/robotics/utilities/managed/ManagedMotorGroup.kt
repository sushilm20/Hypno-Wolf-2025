package org.riverdell.robotics.utilities.managed

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import io.liftgate.robotics.mono.states.StateResult
import java.util.concurrent.CompletableFuture
import kotlin.math.abs

class ManagedMotorGroup(
    stateHolder: StateHolder,
    var pid: PIDCoefficients, // P, I, D gains
    var kV: Double = 0.1, // velocity gain
    var kA: Double = 0.1, // acceleration gain
    var kStatic: Double = 0.1, // additive feedforward
    /**
     * Inputs: Measured position, measured velocity
     */
    private var kF: (Double, Double?) -> Double = { _, _ -> 0.0 },
    private val master: DcMotorEx,
    private val slaves: List<DcMotorEx> = listOf()
)
{
    private var stuckProtection: StuckProtection? = null
    private var previousPosition: Int = 0

    private var stable = System.currentTimeMillis()

    private val pidfBuilder = {
        PIDFController(pid, kV, kA, kStatic, kF)
    }

    private var pidfController = pidfBuilder()

    /**
     * No PID updates.
     */
    private var idle = false

    private val state by stateHolder.state<Int>(
        write = {
            val currentPosition = master.currentPosition
            stable = System.currentTimeMillis()

            master.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            slaves.forEach { slave ->
                slave.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }

            pidfController.targetPosition = it.toDouble()
            idle = false
        },
        read = {
            master.currentPosition
        },
        complete = { current, target ->
            (current == target || if (stuckProtection == null)
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
            })
        }
    )

    fun resetEncoders()
    {
        master.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slaves.forEach { slave ->
            slave.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }
    }

    init
    {
        master.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        slaves.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        /**
         * Continuously update the power of the motor to keep it at the
         * desired target value set in the [PIDFController].
         */
        fun generatePower(current: Int): Double?
        {
            if (idle)
            {
                return null
            }

            val velocity = master.velocity

            return pidfController
                .update(
                    measuredPosition = current.toDouble(),
                    measuredVelocity = velocity
                )
                .coerceIn(-1.0, 1.0)
        }

        if (slaves.isEmpty())
        {
            state.additionalPeriodic { current, _ ->
                val power = generatePower(current)
                    ?: return@additionalPeriodic

                master.power = power
            }
        } else
        {
            state.additionalPeriodic { current, _ ->
                val power = generatePower(current)
                    ?: return@additionalPeriodic

                master.power = power
                slaves.forEach { slave ->
                    slave.power = power
                }
            }
        }
    }

    fun rebuild()
    {
        pidfController = pidfBuilder()
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

    fun configureMotors(block: DcMotorEx.() -> Unit) = apply {
        master.block()
        slaves.forEach(block)
    }

    fun isTravelling() = state.inProgress()

    private var timeout = 2000L
    fun withTimeout(timeout: Long) = apply { this.timeout = timeout }

    /**
     * Overrides current state and goes to a target position.
     */
    fun goTo(target: Int): CompletableFuture<StateResult>
    {
        exitIdle()
        return state.override(target, timeout = timeout)
    }

    /**
     * Re-activates PID.
     */
    fun exitIdle()
    {
        idle = false
    }

    /**
     * Releases motor powers and stops all PID activity.
     */
    fun idle()
    {
        if (idle)
        {
            return
        }

        state.reset()
        pidfController.reset()
        idle = true

        master.power = 0.0
        slaves.forEach {
            it.power = 0.0
        }
    }
}