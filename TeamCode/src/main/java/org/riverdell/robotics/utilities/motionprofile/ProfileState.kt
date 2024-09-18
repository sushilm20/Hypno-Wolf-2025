package org.riverdell.robotics.utilities.motionprofile

import kotlinx.serialization.Serializable

@Serializable
data class ProfileState @JvmOverloads constructor(val target: Double = 0.0, val velocity: Double = 0.0, val acceleration: Double = 0.0)