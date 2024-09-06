package org.riverdell.robotics.autonomous.movement.purepursuit

import org.riverdell.robotics.autonomous.geometry.Pose

data class PathAlgorithm @JvmOverloads constructor(
    val targetCompute: (_root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose) -> _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose,
    val pathComplete: (_root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose, _root_ide_package_.org.riverdell.robotics.autonomous.geometry.Pose) -> Boolean,
    var euclideanCompletionCheck: Boolean = false
)