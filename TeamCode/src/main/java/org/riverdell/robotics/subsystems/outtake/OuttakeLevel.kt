package org.riverdell.robotics.subsystems.outtake

import org.riverdell.robotics.subsystems.slides.LiftConfig

enum class OuttakeLevel(val encoderPercentage: Double)
{
    Rest(0.0), Bar2(0.58), HighBasket(1.0);

    val encoderLevel: Int
        get() = (encoderPercentage * LiftConfig.MAX_EXTENSION).toInt()

    fun next() = entries.getOrNull(ordinal + 1)
    fun previous() = entries.getOrNull(ordinal - 1)
}