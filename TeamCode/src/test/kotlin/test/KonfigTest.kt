package test

import io.liftgate.robotics.mono.konfig.konfig
import org.junit.jupiter.api.Test
import org.junit.runner.RunWith
import org.mockito.junit.MockitoJUnitRunner
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