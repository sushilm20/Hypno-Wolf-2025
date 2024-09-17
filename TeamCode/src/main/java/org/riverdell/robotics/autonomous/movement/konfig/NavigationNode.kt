package org.riverdell.robotics.autonomous.movement.konfig

import com.charleskorn.kaml.YamlComment
import com.google.gson.annotations.SerializedName
import kotlinx.serialization.Serializable

@Serializable
data class NavigationNode(
    @SerializedName("mode")
    val navigationMode: NavigationMode = NavigationMode.GoToPosition,
    @YamlComment(
        "You can define waypoints as either Pure Pursuit waypoints with a radius, or just the waypoint itself for GoToPosition or GVF.",
        "Normal Pose waypoints can be defined either explicitly:",
        "x: 14.0, y: 6.0, heading: 0.24",
        "",
        "referencing a point in the \"globals\" Konfig file:",
        "&point1, heading: 0.24",
        "",
        "or a pose in the \"globals\" Konfig file:",
        "&pose1",
        "",
        "---",
        "Pure Pursuit waypoints can be defined using:",
        "x: 14.0, y: 6.0, heading: 0.24, radius: 20.0",
        "",
        "---",
        "Actions can be defined either as a Pure Pursuit ActionWaypoint or just referencing an action definition.",
        "They can be defined as the following:",
        "*action_id",
        "",
        "For pure pursuit, you may NOT have an action waypoint start your path.",
        "---",
        "Start your path below!"
    )
    val waypoints: Set<String> = setOf(
        """
                point:
                  x: 6.0
                  y: 6.0
                radius: 20.0
                type: "POSE"
            """.trimIndent()
    ),
    @SerializedName("stuck-recovery")
    val stuckProtection: InternalStuckProtectionConfig = InternalStuckProtectionConfig(),
    @SerializedName("maximum-translation-speed")
    val maxTransationalSpeed: Double = 1.0,
    @SerializedName("maximum-rotation-speed")
    val maxRotationalSpeed: Double = 1.0,
    @SerializedName("automatic-death-after-millis")
    val automaticDeathMillis: Double = 2500.0
)
{
    fun sanitize(globals: AutonomousDefaults)
    {
        // {&pose1,"radius":20.0}
        waypoints.
    }
}