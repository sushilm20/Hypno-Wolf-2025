package org.riverdell.robotics.autonomous.movement.guidedvectorfield

import kotlinx.serialization.Serializable
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@Serializable
class Vector2D(val x: Double, val y: Double)
{
    val heading: Double
        get() = atan2(y, x)
    val magnitude: Double
        get() = hypot(x, y)
    val magSq: Double
        get() = x * x + y * y

    // Operations
    fun add(other: Vector2D): Vector2D
    {
        return add(this, other)
    }

    fun subtract(other: Vector2D): Vector2D
    {
        return subtract(this, other)
    }

    fun scalarMultiply(scalar: Double): Vector2D
    {
        return scalarMultiply(this, scalar)
    }

    fun scalarDivide(scalar: Double): Vector2D
    {
        return scalarDivide(this, scalar)
    }

    fun polar(r: Double, t: Double): Vector2D
    {
        return Vector2D(r * cos(t), r * sin(t))
    }

    fun add(a: Vector2D, b: Vector2D): Vector2D
    {
        return Vector2D(a.x + b.x, a.y + b.y)
    }

    fun subtract(a: Vector2D, b: Vector2D): Vector2D
    {
        return Vector2D(a.x - b.x, a.y - b.y)
    }

    fun scalarMultiply(vec: Vector2D, scalar: Double): Vector2D
    {
        return Vector2D(vec.x * scalar, vec.y * scalar)
    }

    fun scalarDivide(vec: Vector2D, scalar: Double): Vector2D
    {
        return Vector2D(vec.x / scalar, vec.y / scalar)
    }

    fun slerp(a: Vector2D, b: Vector2D, t: Double): Vector2D
    {
        val aMag = a.magnitude
        val aHead = a.heading
        val bMag = b.magnitude
        val bHead = b.heading
        return polar((1 - t) * aMag + t * bMag, (1 - t) * aHead + t * bHead)
    }
}