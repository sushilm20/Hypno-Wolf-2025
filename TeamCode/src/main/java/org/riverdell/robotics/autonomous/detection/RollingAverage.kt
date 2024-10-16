package org.riverdell.robotics.autonomous.detection

import kotlin.math.abs

class RollingAverage(private val windowSize: Int)
{
    private val window = ArrayDeque<Double>()
    private var sum = 0.0
    fun add(value: Double): Double
    {
        window.addLast(value)
        sum += value
        if (window.size > windowSize)
        {
            sum -= window.removeFirst()
        }

        return getAverage()
    }

    fun getAverage() = if (window.isNotEmpty()) sum / window.size else 0.0
    fun isAvailable() = window.size == windowSize

    fun isOutlier(area: Double, threshold: Double) = abs(area - getAverage()) > threshold

    fun reset()
    {
        window.clear()
        sum = 0.0
    }
}