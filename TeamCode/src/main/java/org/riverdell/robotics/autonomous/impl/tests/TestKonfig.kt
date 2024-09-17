package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.konfig.konfig
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.AutonomousWrapper
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigatePurePursuit
import org.riverdell.robotics.autonomous.movement.navigateToPosition
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.PositionWaypoint

@Autonomous(name = "Test | Konfig Hot Reload", group = "Test")
class TestKonfig : AutonomousWrapper({ opMode ->
    opMode.navigationConfig.onHotReload {
        println("Test hot reload")
    }

    Thread.sleep(10000L)
})