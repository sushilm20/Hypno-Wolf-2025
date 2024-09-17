package test

import com.charleskorn.kaml.Yaml
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import org.junit.jupiter.api.Test
import org.junit.runner.RunWith
import org.mockito.junit.MockitoJUnitRunner
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNode
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNodeCollection
import org.riverdell.robotics.autonomous.movement.purepursuit.PoseWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.PositionWaypoint

@RunWith(MockitoJUnitRunner::class)
class KonfigTest
{
    @Test
    fun whatYamlFormatAmIExpecting()
    {
        // point:,  x: 6.0,  y: 6.0,radius: 20.0,type: POSE
        println(
            Json.encodeToString(PositionWaypoint(Pose(6.0, 6.0, 6.0.degrees), 20.0))
        )

        println(
            Json.encodeToString(PoseWaypoint(Pose(6.0, 6.0, 6.0.degrees), 20.0))
        )

        println(
            Json.encodeToString(Pose(6.0, 6.0, 6.0.degrees))
        )
    }
}