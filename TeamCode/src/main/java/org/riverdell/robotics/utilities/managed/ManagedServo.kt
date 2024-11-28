package org.riverdell.robotics.utilities.managed

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.motionprofile.AsymmetricMotionProfile
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints
import org.riverdell.robotics.utilities.motionprofile.ProfileState
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
) {
    private var motionProfile: AsymmetricMotionProfile? = null
    private var timer = ElapsedTime()
    private var behavior = ServoBehavior.MotionProfile
    private var profileState: ProfileState? = null

    private val state by stateHolder.state<Double>({
        timer = ElapsedTime()
        if (behavior == ServoBehavior.MotionProfile)
        {
            println("Motion profile going from ${servo.position} to $it")
            motionProfile = AsymmetricMotionProfile(
                servo.position,
                it,
                constraints()
            )
        } else
        {
            motionProfile = null
        }
    }, { _ ->
        servo.position
    }, { _, target ->
        if (behavior == ServoBehavior.Direct) {
            servo.position = target
            return@state true
        } else
        {
            profileState = motionProfile!!.calculate(timer.time())
            if (profileState!!.v == 0.0) {
                return@state true
            }

            servo.position = profileState!!.x
            return@state false
        }
    })

    fun unwrapServo() = servo
    fun setTarget(
        target: Double,
        behavior: ServoBehavior = ServoBehavior.MotionProfile
    ): CompletableFuture<*> {
        this.behavior = behavior
        return state.override(target)
    }

    fun cancelMotionProfile() = state.reset()
}