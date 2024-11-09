package org.riverdell.robotics.subsystems.outtake

enum class OuttakeLevel(val encoderLevel: Int)
{
    Bar1(170), Bar2(780), HighBasket(1800);

    fun next() = entries.getOrNull(ordinal + 1)
    fun previous() = entries.getOrNull(ordinal - 1)
}