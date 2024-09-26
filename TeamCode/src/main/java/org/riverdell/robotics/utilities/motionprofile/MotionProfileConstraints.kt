package org.riverdell.robotics.utilities.motionprofile

import kotlinx.serialization.Serializable
import kotlin.math.abs

@Serializable
data class MotionProfileConstraints(var velocity: Double, var acceleration: Double, var deceleration: Double)
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