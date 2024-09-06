package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.simultaneous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.AutonomousWrapper
import org.riverdell.robotics.autonomous.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.lockToPosition

@Autonomous(name = "Test | Lock", group = "Test")
class TestLock : AutonomousWrapper({ opMode, _ ->
    simultaneous("cone drop simulation") {
        var hasConeBeenDropped = false
        single("drop cone") {
            Thread.sleep(10000L)
            hasConeBeenDropped = true
        }

        lockToPosition(Pose(0.0, 0.0, 0.0.degrees)) { _, _ ->
            hasConeBeenDropped
        }
    }
})