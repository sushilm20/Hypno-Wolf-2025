package test

import com.charleskorn.kaml.AmbiguousQuoteStyle
import com.charleskorn.kaml.SingleLineStringStyle
import com.charleskorn.kaml.Yaml
import com.charleskorn.kaml.YamlConfiguration
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
        println(Yaml(configuration = YamlConfiguration(ambiguousQuoteStyle = AmbiguousQuoteStyle.SingleQuoted, singleLineStringStyle = SingleLineStringStyle.PlainExceptAmbiguous)).encodeToString(NavigationNodeCollection(
            nodes = mapOf("main" to NavigationNode())
        )))
    }
}