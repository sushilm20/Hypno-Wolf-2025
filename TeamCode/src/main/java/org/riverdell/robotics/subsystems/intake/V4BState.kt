package org.riverdell.robotics.subsystems.intake

enum class V4BState(val position: Double)
{
    Lock(0.0),
    UnlockedIdleHover(0.07),
    Intermediate(0.3),
    Select(0.50),
    Pickup(0.75)
}