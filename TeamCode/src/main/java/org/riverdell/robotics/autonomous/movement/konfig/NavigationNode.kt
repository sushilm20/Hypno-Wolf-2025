package org.riverdell.robotics.autonomous.movement.konfig

import com.google.gson.annotations.SerializedName
import io.liftgate.robotics.mono.pipeline.BacktrackException
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import net.mamoe.yamlkt.Comment
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import java.util.concurrent.atomic.AtomicInteger

val degreesFunction = """deg\(([^)]+)\)""".toRegex()
val referenceRegex = """&[^,\s{}]+""".toRegex()

val globalJson = Json {
    ignoreUnknownKeys = true
}

@Serializable
data class NavigationNode(
    @SerializedName("mode")
    val navigationMode: NavigationMode = NavigationMode.GoToPosition,
    val waypoints: Set<String> = setOf(
        "{\"x\":6.0,\"y\":6.0,\"heading\":0.10471975511965978}",
        "{\"x\":6.0,\"y\":6.0,\"heading\":deg(20.0)}",
        "{&point1,\"heading\":0.10471975511965978}",
        "*pickup_sample",
        "&pose1",
        "{\"x\":8.0,\"y\":9.0,\"heading\":deg(40.0)}",
    ),
    @SerializedName("stuck-recovery")
    val stuckProtection: InternalStuckProtectionConfig = InternalStuckProtectionConfig(),
    @SerializedName("maximum-translation-speed")
    val maxTransationalSpeed: Double = 1.0,
    @SerializedName("maximum-rotation-speed")
    val maxRotationalSpeed: Double = 1.0,
    val automaticDeathEnabled: Boolean = true,
    @SerializedName("automatic-death-after-millis")
    val automaticDeathMillis: Double = 2500.0
)
{
    fun applyToPositionChange(executionGroup: RootExecutionGroup, positionChangeAction: PositionChangeAction)
    {
        positionChangeAction.withCustomMaxRotationalSpeed(maxRotationalSpeed)
        positionChangeAction.withCustomMaxTranslationalSpeed(maxTransationalSpeed)
        if (!automaticDeathEnabled)
        {
            positionChangeAction.disableAutomaticDeath()
        } else
        {
            positionChangeAction.withAutomaticDeath(automaticDeathMillis)
        }

        if (stuckProtection.enabled)
        {
            if (!stuckProtection.recoveryAction.shouldBackTrack)
            {
                positionChangeAction.whenStuck(stuckProtection.stuckProtection) {
                    executionGroup.terminateMidExecution()
                }
                return
            }

            positionChangeAction.whenStuckWithMaxAttempts(
                stuckProtection.stuckProtection,
                AtomicInteger(),
                stuckProtection.recoveryAction.backTrackAmount
            ) {
                throw BacktrackException(stuckProtection.recoveryAction.backTrackAmount)
            }
        }
    }

    fun prepareForApplication(globals: AutonomousDefaults) = waypoints
        .map { unparsed ->
            var it = unparsed
            val matchResult = degreesFunction.find(it)
            if (matchResult != null)
            {
                val group = matchResult.groupValues[1]
                val toRadians = AngleUnit.normalizeRadians(group.toDouble())
                it = degreesFunction.replace(it, "$toRadians")
            }

            val extracted = referenceRegex.find(it)
                ?: return@map it

            val value = extracted.value
            val pose = globals.poses[value.removePrefix("&")]
            val point = globals.points[value.removePrefix("&")]

            if (it.contains("heading"))
            {
                if (pose != null)
                {
                    return@map it.replace(referenceRegex, "\"pose\":${
                        globalJson.encodeToString(pose)
                    }")
                } else if (point != null)
                {
                    return@map it.replace(referenceRegex, "\"point\":${
                        globalJson.encodeToString(point)
                    }")
                } else
                {
                    throw IllegalArgumentException("Failed to parse $it as a Pure Pursuit waypoint as no reference pose/point was found. Skipping.")
                }
            } else
            {
                if (it.startsWith("{"))
                {
                    if (pose != null)
                    {
                        return@map globalJson.encodeToString(pose)
                    } else if (point != null)
                    {
                        return@map it.replace(referenceRegex, "\"x\": ${point.x}, \"y\": ${point.y}")
                    }
                } else
                {
                    if (pose != null)
                    {
                        return@map globalJson.encodeToString(pose)
                    } else if (point != null)
                    {
                        return@map globalJson.encodeToString(Pose(point.x, point.y, 0.0.degrees))
                    }
                }

                throw IllegalArgumentException("The pose/point referenced in $it does not exist.")
            }
        }
}