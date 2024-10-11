package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.riverdell.robotics.autonomous.detection.VisionPipeline

@TeleOp(
    name = "Sample Detection Test",
    group = "Tests"
)
class TestSampleDetection : LinearOpMode()
{
    val visionPipeline by lazy { VisionPipeline(this) }
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        visionPipeline.start()

        while (opModeIsActive())
        {
            telemetry.addLine("hi")
            telemetry.update()
            Thread.sleep(50L)
        }

        visionPipeline.dispose()
    }
}