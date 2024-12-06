package org.riverdell.robotics.subsystems.outtake

enum class OuttakeCoaxialState(val position: Double)
{
    Ready(1.0), Transfer(0.8), Deposit(0.55), OutsideIntake(0.55)
}