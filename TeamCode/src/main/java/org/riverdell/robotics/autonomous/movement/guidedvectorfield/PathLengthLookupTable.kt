package org.riverdell.robotics.autonomous.movement.guidedvectorfield

import org.riverdell.robotics.autonomous.movement.geometry.CubicBezierCurve

class PathLengthLookupTable(
    private val curve: CubicBezierCurve,
    sampleDensity: Int
)
{
    private val intervals: MutableList<Double> = mutableListOf()
    private val cumulativeLengths: MutableList<Double> = mutableListOf()

    init
    {
        computeIntervals(sampleDensity)
    }

    private fun computeIntervals(sampleDensity: Int)
    {
        var totalLength = 0.0
        var previousPoint = curve.calculate(0.0)
        var intervalLength: Double

        for (i in 1..sampleDensity)
        {
            val t = i / sampleDensity.toDouble()
            val currentPoint = curve.calculate(t)
            intervalLength = currentPoint.subtract(previousPoint).magnitude
            intervals.add(intervalLength)
            totalLength += intervalLength
            cumulativeLengths.add(totalLength)
            previousPoint = currentPoint
        }
    }

    fun getCumulativeLength(t: Double): Double
    {
        val index = (t * intervals.size).toInt().coerceAtMost(cumulativeLengths.size - 1)
        return cumulativeLengths[index]
    }
}
