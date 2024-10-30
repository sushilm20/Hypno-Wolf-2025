package org.riverdell.robotics.utilities.managed

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture

/**
 * A [Servo] wrapper that keeps track of motion profile states.
 *
 * @author Subham
 */
class ManagedServo(
    private val servo: ServoImplEx,
    stateHolder: StateHolder,
    private val constraints: () -> ProfileConstraints
)
{
    private var motionProfile: AsymmetricMotionProfile? = null
    private var timer = ElapsedTime()

    private val state by stateHolder.state<Double>({
        timer = ElapsedTime()
        motionProfile = AsymmetricMotionProfile(
            servo.position,
            it,
            constraints()
        )
    }, {
        if (motionProfile == null)
        {
            return@state 0.0
        }

        val motionProfileState = motionProfile!!.calculate(timer.time())
        servo.position = motionProfileState.x
        servo.position
    }, { current, finalPosition ->
        if (motionProfile == null)
        {
            return@state false
        }

        if (current == finalPosition)
        {
            motionProfile = null
        }

        motionProfile == null
    })

    fun unwrapServo() = servo
    fun setMotionProfileTarget(target: Double) = state.override(target, 0L)
    fun cancelMotionProfile() = state.reset()

    /**
     * Overrides any existing motion profile and sets
     * the target position of the backing servo.
     */
    fun forcefullySetTarget(targetPosition: Double): CompletableFuture<Void>
    {
        state.reset()
        servo.position = targetPosition
        return CompletableFuture.completedFuture(null)
    }
}