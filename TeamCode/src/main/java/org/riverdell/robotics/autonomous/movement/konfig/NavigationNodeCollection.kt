package org.riverdell.robotics.autonomous.movement.konfig

import kotlinx.serialization.Serializable

@Serializable
data class NavigationNodeCollection(
    val nodes: Map<String, NavigationNode> = mapOf("main" to NavigationNode())
)