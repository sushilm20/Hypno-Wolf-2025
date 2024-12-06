package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.75), Transfer(0.85), Deposit(0.60), Force(0.3)
}