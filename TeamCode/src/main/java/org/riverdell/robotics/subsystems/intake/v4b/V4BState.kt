package org.riverdell.robotics.subsystems.intake.v4b

enum class V4BState(val position: Double)
{
    Lock(0.19),
    UnlockedIdleHover(0.4),
    Transfer(0.25),
    Intermediate(0.5),
    Gateway(0.85),
//    Focus(0.73),
    Pickup(0.935)
}