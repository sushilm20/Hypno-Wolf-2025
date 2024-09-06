package org.riverdell.robotics.autonomous.movement.purepursuit

import org.riverdell.robotics.autonomous.geometry.Pose

data class PathAlgorithm @JvmOverloads constructor(
    val targetCompute: (Pose) -> Pose,
    val pathComplete: (Pose, Pose) -> Boolean,
    var euclideanCompletionCheck: Boolean = false
)