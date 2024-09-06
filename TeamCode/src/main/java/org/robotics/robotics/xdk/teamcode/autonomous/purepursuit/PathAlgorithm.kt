package org.robotics.robotics.xdk.teamcode.autonomous.purepursuit

import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose

data class PathAlgorithm @JvmOverloads constructor(
    val targetCompute: (Pose) -> Pose,
    val pathComplete: (Pose, Pose) -> Boolean,
    var euclideanCompletionCheck: Boolean = false
)