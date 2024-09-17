package org.riverdell.robotics.autonomous.movement.konfig

import com.charleskorn.kaml.YamlComment
import com.google.gson.annotations.SerializedName
import io.liftgate.robotics.mono.pipeline.BacktrackException
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import java.util.concurrent.atomic.AtomicInteger

val referenceRegex = """&[^,\s{}]+""".toRegex()
val globalJson = Json {
    ignoreUnknownKeys = true
}

@Serializable
data class NavigationNode(
    @SerializedName("mode")
    val navigationMode: NavigationMode = NavigationMode.GoToPosition,
    @YamlComment(
        "You can define waypoints as either Pure Pursuit waypoints with a radius, or just the waypoint itself for GoToPosition or GVF.",
        "Normal Pose waypoints can be defined either explicitly:",
        "{\"x\":6.0,\"y\":6.0,\"heading\":0.10471975511965978}",
        "",
        "or referencing a pose/point in the \"globals\" Konfig file:",
        "{&point1,\"heading\":0.10471975511965978}",
        "",
        "or a pose in the \"globals\" Konfig file:",
        "&pose1",
        "",
        "---",
        "Pure Pursuit waypoints can be defined using:",
        "{\"pose\": {\"x\": 14.0, \"y\": 6.0, \"heading\": 14.0}, \"radius\": 20.0}",
        "{\"point\": {\"x\": 14.0, \"y\": 6.0}, \"radius\": 20.0}",
        "",
        "You can use the \"&\" syntax similarly in Pure Pursuit waypoints as we did before.",
        "{&pose1, \"radius\": 20.0}",
        "{&point1, \"radius\": 20.0}",
        "",
        "---",
        "Actions can be defined either as a Pure Pursuit ActionWaypoint or just referencing an action definition.",
        "They can be defined as the following:",
        "*action_id",
        "",
        "For pure pursuit, you may NOT have an action waypoint start your path, and you must have a POSE to end your path.",
        "---",
        "Start your path below!"
    )
    val waypoints: Set<String> = setOf(
        "{\"x\":6.0,\"y\":6.0,\"heading\":0.10471975511965978}",
        "{&point1,\"heading\":0.10471975511965978}",
        "&pose1"
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
        .map {
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