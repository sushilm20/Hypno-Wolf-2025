package org.riverdell.robotics.utilities.managed

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints

/**
 * A [Servo] wrapper that keeps track of motion profile states.
 *
 * @author Subham
 */
class ManagedServo(
    private val servo: Servo,
    stateHolder: StateHolder,
    private val constraints: () -> _root_ide_package_.org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
)
{
    private var motionProfile: _root_ide_package_.org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile? = null
    private val timer = ElapsedTime()

    private val state by stateHolder.state<Double>({
        motionProfile =
            _root_ide_package_.org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile(
                servo.position,
                it,
                constraints()
            )
        timer.reset()
    }, {
        val position = motionProfile?.calculate(timer.time())
            ?: return@state servo.position
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
    fun forcefullySetTarget(targetPosition: Double)
    {
        state.reset()
        servo.position = targetPosition
    }
}