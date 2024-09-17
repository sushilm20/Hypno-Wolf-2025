package test

import org.junit.jupiter.api.Test
import org.junit.runner.RunWith
import org.mockito.junit.MockitoJUnitRunner
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.konfig.AutonomousDefaults
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNode

@RunWith(MockitoJUnitRunner::class)
class KonfigTest
{
    @Test
    fun whatYamlFormatAmIExpecting()
    {
        println(NavigationNode().prepareForApplication(AutonomousDefaults(
            poses = mapOf("pose1" to Pose(6.0, 9.0, 1.0.degrees)),
            points = mapOf("point1" to Point(6.0, 9.0)),
        )))
    }
}