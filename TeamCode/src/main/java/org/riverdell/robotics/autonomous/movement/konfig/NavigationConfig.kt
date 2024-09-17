package org.riverdell.robotics.autonomous.movement.konfig

import kotlinx.serialization.Serializable

@Serializable
class NavigationConfig(
    val kStatic: Double = 1.85,
    val xP: Double = 0.07,
    val xD: Double = 0.012,
    val yP: Double = 0.07,
    val yD: Double = 0.012,
    val hP: Double = 1.0,
    val hD: Double = 0.075,
    val minimumTranslationDifferenceFromTarget: Double = 0.75,
    val minimumRotationalDifferenceFromTarget: Double = 0.02,
    val automaticDeathMillis: Double = 100.0
)