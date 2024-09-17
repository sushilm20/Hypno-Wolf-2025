package org.riverdell.robotics.autonomous.movement.konfig

import com.google.gson.annotations.SerializedName
import kotlinx.serialization.Serializable
import org.riverdell.robotics.autonomous.movement.RobotStuckProtection

@Serializable
data class InternalStuckProtectionConfig(
    val enabled: Boolean = false,
    @SerializedName("configuration")
    val stuckProtection: RobotStuckProtection = RobotStuckProtection(),
    @SerializedName("backtrack-recovery")
    val recoveryAction: StuckRecoveryAction = StuckRecoveryAction()
)
