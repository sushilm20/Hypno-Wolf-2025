package org.riverdell.robotics.utilities.managed

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.liftgate.robotics.mono.states.StateHolder
import kotlin.math.abs

class ManagedMotorGroup(
    stateHolder: StateHolder,
    var pid: PIDCoefficients,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kStatic: Double = 0.0,
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

    private val state by stateHolder.state<Int>(
        write = {
            stable = System.currentTimeMillis()
            pidfController.targetPosition = it.toDouble()
            println("Writing target position to ${it.toDouble()}")
        },
        read = {
            master.currentPosition.apply {
                println("reading current position: at $this")
            }
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
            }).apply {
                println("State is complete? $this")
            }
        }
    )

    init
    {
        master.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        slaves.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        println("Updated motors for the master to use W/O encoder")

        /**
         * Continuously update the power of the motor to keep it at the
         * desired target value set in the [PIDFController].
         */
        if (slaves.isEmpty())
        {
            state.additionalPeriodic { current, _ ->
                val velocity = master.velocity
                master.power = pidfController
                    .update(
                        measuredPosition = current.toDouble(),
                        measuredVelocity = velocity
                    )
            }
        } else
        {
            println("Slave motor count: ${slaves.size}")
            state.additionalPeriodic { current, _ ->
                val velocity = master.velocity // TODO: avg velocity? no clue
                val pidf = pidfController
                    .update(
                        measuredPosition = current.toDouble(),
                        measuredVelocity = velocity
                    )

                master.power = pidf
                slaves.forEach { slave ->
                    slave.power = pidf
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
    fun goTo(target: Int) = state.override(target)
    fun reset() = state.reset()
}