package test

import com.charleskorn.kaml.AmbiguousQuoteStyle
import com.charleskorn.kaml.SingleLineStringStyle
import com.charleskorn.kaml.Yaml
import com.charleskorn.kaml.YamlConfiguration
import io.liftgate.robotics.mono.konfig.konfig
import kotlinx.serialization.encodeToString
import org.junit.jupiter.api.Test
import org.junit.runner.RunWith
import org.mockito.junit.MockitoJUnitRunner
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNode
import org.riverdell.robotics.autonomous.movement.konfig.NavigationNodeCollection
import org.riverdell.robotics.utilities.managed.pidf.PIDFMotionProfiledConfig

@RunWith(MockitoJUnitRunner::class)
class KonfigTest
{
    @Test
    fun whatYamlFormatAmIExpecting()
    {
        val konfig = konfig<PIDFMotionProfiledConfig> { local(); withCustomFileID("testpid_motionprofile") }
        println(konfig.get())
        println(konfig<PIDFMotionProfiledConfig> { local(); withCustomFileID("testpid_motionprofile") }.get())
    }
}