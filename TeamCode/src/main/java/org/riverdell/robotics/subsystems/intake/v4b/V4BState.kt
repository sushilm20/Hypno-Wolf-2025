package org.riverdell.robotics.subsystems.intake.v4b

enum class V4BState(val position: Double)
{
    Lock(0.0),
    UnlockedIdleHover(0.07),
    Transfer(0.04),
    Intermediate(0.3),
    Gateway(0.7),
//    Focus(0.73),
    Pickup(0.75)
}