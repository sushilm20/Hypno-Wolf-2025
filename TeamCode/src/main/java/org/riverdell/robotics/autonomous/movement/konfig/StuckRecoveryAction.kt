package org.riverdell.robotics.autonomous.movement.konfig

import com.google.gson.annotations.SerializedName
import kotlinx.serialization.Serializable
import net.mamoe.yamlkt.Comment

@Serializable
data class StuckRecoveryAction(
    @SerializedName("enabled")
    val shouldBackTrack: Boolean = true,
    @SerializedName("back-track-amount")
    @Comment(
        "Amount of stages to go back, to try to re-attempt navigation."
    )
    val backTrackAmount: Int = 1,
    @SerializedName("termination-threshold")
    @Comment(
        "Amount of back track attempts until the navigation terminates."
    )
    val terminateAfterAttempts: Int = 3
)