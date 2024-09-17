package org.riverdell.robotics.autonomous.movement.geometry

import kotlinx.serialization.Serializable
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.Vector2D

@Serializable
data class CubicBezierCurve(
    val first: Vector2D,
    val second: Vector2D,
    val third: Vector2D,
    val fourth: Vector2D
)
{
    fun calculate(t: Double): Vector2D
    {
        // (1 - t)^3 * P0 + 3 * t * (1 - t)^2 * P1 + 3 * t^2 * (1 - t) * P2 + t^3 * P3
        val w = 1 - t
        val firstTerm = first.scalarMultiply(w * w * w)
        val secondTerm = second.scalarMultiply(3 * t * w * w)
        val thirdTerm = third.scalarMultiply(3 * t * t * w)
        val fourthTerm = fourth.scalarMultiply(t * t * t)
        return firstTerm.add(secondTerm).add(thirdTerm).add(fourthTerm)
    }

    fun derivative(t: Double): Vector2D
    {
        val w = 1 - t
        val firstTerm = second.subtract(first).scalarMultiply(3 * w * w)
        val secondTerm = third.subtract(second).scalarMultiply(6 * w * t)
        val thirdTerm = fourth.subtract(third).scalarMultiply(3 * t * t)
        return firstTerm.add(secondTerm).add(thirdTerm)
    }

    fun slope(t: Double): Double
    {
        val dt = derivative(t)
        return dt.y / dt.x
    }

    fun heading(t: Double): Double
    {
        return derivative(t).heading
    }
}