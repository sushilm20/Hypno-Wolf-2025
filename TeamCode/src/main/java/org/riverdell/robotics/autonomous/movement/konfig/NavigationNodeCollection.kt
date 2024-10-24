package org.riverdell.robotics.autonomous.movement.konfig

class NavigationNodeCollection(
    val nodes: Map<String, NavigationNode> = mapOf("main" to NavigationNode())
)