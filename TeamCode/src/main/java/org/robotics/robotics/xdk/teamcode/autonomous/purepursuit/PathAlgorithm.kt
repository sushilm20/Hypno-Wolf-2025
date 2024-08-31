package org.robotics.robotics.xdk.teamcode.autonomous.purepursuit

import org.robotics.robotics.xdk.teamcode.autonomous.geometry.Pose

data class PathAlgorithm(
    val targetCompute: (Pose) -> Pose,
    val pathComplete: (Pose, Pose) -> Boolean
)