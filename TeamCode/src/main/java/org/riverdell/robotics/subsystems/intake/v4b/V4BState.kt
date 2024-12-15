package org.riverdell.robotics.subsystems.intake.v4b

enum class V4BState(val position: Double)
{
    Lock(0.21),
    UnlockedIdleHover(0.4),
    Transfer(0.23),
    Intermediate(0.5), //0.5
    AutoGateway(0.8),

    Gateway(0.9),
//    Focus(0.73),
    Pickup(0.975) //0.935
}