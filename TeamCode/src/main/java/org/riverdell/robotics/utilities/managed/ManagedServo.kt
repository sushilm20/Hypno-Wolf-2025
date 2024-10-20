package org.riverdell.robotics.utilities.managed

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile
import org.riverdell.robotics.utilities.motionprofile.MotionProfileConstraints

/**
 * A [Servo] wrapper that keeps track of motion profile states.
 *
 * @author Subham
 */
class ManagedServo(
    private val servo: Servo,
    stateHolder: StateHolder,
    private val constraints: () -> MotionProfileConstraints
)
{
    private var motionProfile: AsymmetricMotionProfile? = null
    private val timer = ElapsedTime()

    private val state by stateHolder.state<Double>({
        motionProfile =
            AsymmetricMotionProfile(
                servo.position,
                it,
                constraints()
            )
        timer.reset()
    }, {
        val servoCurrentPosition = servo.position
        val motionProfileState = motionProfile?.calculate(timer.time())
            ?: return@state servoCurrentPosition

        if (servoCurrentPosition == motionProfileState.target)
        {
            motionProfile = null
            return@state servoCurrentPosition
        }

        servo.position = motionProfileState.target
        motionProfileState.target
    })

    fun unwrapServo() = servo
    fun setMotionProfileTarget(target: Double) = state.override(target, timeout = Long.MAX_VALUE)
    fun cancelMotionProfile() = state.reset()

    /**
     * Overrides any existing motion profile and sets
     * the target position of the backing servo.
     */
    fun forcefullySetTarget(targetPosition: Double)
    {
        state.reset()
        servo.position = targetPosition
    }
}