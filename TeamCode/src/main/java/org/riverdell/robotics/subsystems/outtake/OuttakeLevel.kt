package org.riverdell.robotics.subsystems.outtake

import org.riverdell.robotics.subsystems.slides.LiftConfig

enum class OuttakeLevel(val encoderPercentage: Double)
{
    Bar1(0.2), Bar2(0.5), HighBasket(1.0);

    val encoderLevel: Double
        get() = encoderPercentage * LiftConfig.MAX_EXTENSION

    fun next() = entries.getOrNull(ordinal + 1)
    fun previous() = entries.getOrNull(ordinal - 1)
}