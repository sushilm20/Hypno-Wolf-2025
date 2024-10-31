package org.riverdell.robotics.subsystems.hang

enum class HangState(val power: Double)
{
    Idle(0.0), Powered(1.0)
}