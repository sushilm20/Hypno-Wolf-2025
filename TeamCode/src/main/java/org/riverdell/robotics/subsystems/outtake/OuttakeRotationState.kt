package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.55), Transfer(0.85), Deposit(0.45), Force(0.25)
}