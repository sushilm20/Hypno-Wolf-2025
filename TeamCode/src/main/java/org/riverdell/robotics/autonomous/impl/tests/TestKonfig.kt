package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.HypnoticAuto

@Autonomous(name = "Test | Konfig Hot Reload", group = "Test")
class TestKonfig : HypnoticAuto({ opMode ->
    opMode.navigationConfig.onHotReload {
        println("Test hot reload")
    }

    Thread.sleep(10000L)
})