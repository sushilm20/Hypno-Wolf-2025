package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.movement.konfig.KonfigAutonomous

@Autonomous(name = "Test | Konfig", group = "Test")
class KonfigAutonomousTest : KonfigAutonomous("testing")
{
    override fun definitions()
    {
        action("pickup_sample") {
            println("Pickup sample shit")
        }
    }
}