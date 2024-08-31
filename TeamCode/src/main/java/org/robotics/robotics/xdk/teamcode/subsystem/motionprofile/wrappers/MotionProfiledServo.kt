package org.robotics.robotics.xdk.teamcode.subsystem.motionprofile.wrappers

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.robotics.robotics.xdk.teamcode.subsystem.motionprofile.AsymmetricMotionProfile
import org.robotics.robotics.xdk.teamcode.subsystem.motionprofile.ProfileConstraints

/**
 * A [Servo] wrapper that keeps track of motion profile states.
 *
 * @author Subham
 */
class MotionProfiledServo(
    private val servo: Servo,
    stateHolder: StateHolder,
    private val constraints: () -> ProfileConstraints
)
{
    private var motionProfile: AsymmetricMotionProfile? = null
    private val timer = ElapsedTime()

    private val state by stateHolder.state<Double>({
        motionProfile = AsymmetricMotionProfile(
            servo.position,
            it,
            constraints()
        )
        timer.reset()
    }, {
        val position = motionProfile!!.calculate(timer.time())
        servo.position = position.x
        position.x
    })

    fun unwrapServo() = servo
    fun setMotionProfileTarget(target: Double) = state.override(target)
    fun cancelMotionProfile() = state.reset()

    /**
     * Overrides any existing motion profile and sets
     * the target position of the backing servo.
     */
    fun setTarget(targetPosition: Double)
    {
        state.reset()
        servo.position = targetPosition
    }
}