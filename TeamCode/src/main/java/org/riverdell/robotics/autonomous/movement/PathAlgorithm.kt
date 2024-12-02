package org.riverdell.robotics.autonomous.movement

import org.riverdell.robotics.autonomous.movement.geometry.Pose

data class PathAlgorithm @JvmOverloads constructor(
    val targetCompute: (Pose) -> Pose,
    val pathComplete: (Pose, Pose) -> Boolean,
    var euclideanCompletionCheck: Boolean = false,
    var strict: Boolean = false
)