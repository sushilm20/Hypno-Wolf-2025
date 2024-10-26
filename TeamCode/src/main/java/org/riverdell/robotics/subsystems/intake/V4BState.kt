package org.riverdell.robotics.subsystems.intake

enum class V4BState(val position: Double)
{
    Lock(0.0),
    Intermediate(0.3),
    Select(0.67),
    Pickup(0.73)
}