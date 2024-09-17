package org.riverdell.robotics.autonomous.movement.konfig

import com.charleskorn.kaml.YamlComment
import com.google.gson.annotations.SerializedName
import kotlinx.serialization.Serializable

@Serializable
data class StuckRecoveryAction(
    @SerializedName("enabled")
    val shouldBackTrack: Boolean = true,
    @SerializedName("back-track-amount")
    @YamlComment(
        "Amount of stages to go back, to try to re-attempt navigation."
    )
    val backTrackAmount: Int = 1,
    @SerializedName("termination-threshold")
    @YamlComment(
        "Amount of back track attempts until the navigation terminates."
    )
    val terminateAfterAttempts: Int = 3
)