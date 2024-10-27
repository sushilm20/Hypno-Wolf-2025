package org.riverdell.robotics.subsystems

import io.liftgate.robotics.mono.konfig.Konfig
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.hardware
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints

fun StateHolder.motionProfiledServo(name: String, constraints: ProfileConstraints) = ManagedServo(
    HypnoticRobot.instance.hardware(name),
    this
) { constraints }