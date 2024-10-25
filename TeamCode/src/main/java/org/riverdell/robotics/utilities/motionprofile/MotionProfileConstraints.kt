package org.riverdell.robotics.utilities.motionprofile

import kotlinx.serialization.Serializable
import kotlin.math.abs

@Serializable
data class MotionProfileConstraints(var velocity: Double = 50.0, var acceleration: Double = 5.0, var deceleration: Double = 5.0)
{
    constructor(velocity: Double, acceleration: Double) : this(acceleration, acceleration, velocity)

    init
    {
        this.velocity = abs(velocity)
        this.acceleration = abs(acceleration)
        this.deceleration = abs(deceleration)
    }

    fun multiply(factor: Double): MotionProfileConstraints
    {
        return MotionProfileConstraints(
            velocity * factor,
            acceleration * factor,
            deceleration * factor,
        )
    }
}