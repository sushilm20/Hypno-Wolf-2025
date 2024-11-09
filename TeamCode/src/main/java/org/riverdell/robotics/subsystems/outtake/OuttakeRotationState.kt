package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Transfer(0.1), Deposit(1.0), Force(0.5)
}