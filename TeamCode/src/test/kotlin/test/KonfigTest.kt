package test

import com.charleskorn.kaml.Yaml
import kotlinx.serialization.encodeToString
import org.junit.jupiter.api.Test
import org.junit.runner.RunWith
import org.mockito.junit.MockitoJUnitRunner
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNode
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNodeCollection

@RunWith(MockitoJUnitRunner::class)
class KonfigTest
{
    @Test
    fun whatYamlFormatAmIExpecting()
    {
        println(Yaml().encodeToString(NavigationNodeCollection(mapOf(
            "first" to NavigationNode()
        ))))
    }
}