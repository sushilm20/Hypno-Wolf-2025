package org.riverdell.robotics.utilities.managed

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import java.util.concurrent.CompletableFuture
import kotlin.math.abs

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
    private var target = 0.0

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

        if (current == finalPosition || abs(current - finalPosition) < 0.03)
        {
            motionProfile = null
        }

        motionProfile == null
    })

    fun unwrapServo() = servo
    fun setTarget(target: Double, behavior: ServoBehavior = ServoBehavior.MotionProfile): CompletableFuture<*>
    {
        if (behavior == ServoBehavior.MotionProfile)
        {
            return state.override(target, 1000L)
        }

        servo.position = target
        return CompletableFuture.completedFuture(null)
    }
    fun cancelMotionProfile() = state.reset()
}