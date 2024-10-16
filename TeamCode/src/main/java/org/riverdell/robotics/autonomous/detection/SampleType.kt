package org.riverdell.robotics.autonomous.detection

import org.opencv.core.Scalar

enum class SampleType(
    val colorRangeMinimum: Scalar,
    val colorRangeMaximum: Scalar
) {
    Blue(
        colorRangeMinimum = Scalar(90.0, 100.0, 100.0),
        colorRangeMaximum = Scalar(130.0, 255.0, 255.0)
    ),
    Yellow(
        colorRangeMinimum = Scalar(20.0, 100.0, 100.0),
        colorRangeMaximum = Scalar(30.0, 255.0, 255.0)
    ),
    Red(
        colorRangeMinimum = Scalar(0.0, 100.0, 100.0),
        colorRangeMaximum = Scalar(10.0, 255.0, 255.0)
    )
}
