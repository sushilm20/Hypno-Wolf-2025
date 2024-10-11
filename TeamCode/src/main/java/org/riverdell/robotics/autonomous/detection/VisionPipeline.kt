package org.riverdell.robotics.autonomous.detection

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.riverdell.robotics.utilities.hardware

/**
 * Manages and configures all [VisionPortal] processors
 * for an op mode.
 */
class VisionPipeline(
    private val opMode: LinearOpMode
) : AbstractSubsystem()
{
    private lateinit var portal: VisionPortal
    private lateinit var sampleDetection: SampleDetection

    override fun composeStageContext() = TODO()

    override fun start()
    {
        sampleDetection = SampleDetection()
        portal = VisionPortal.Builder()
            .setCamera(
                opMode.hardware<WebcamName>("webcam")
            )
            .setCameraResolution(Size(1920, 1080))
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .addProcessors(sampleDetection)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        FtcDashboard.getInstance().startCameraStream(
            sampleDetection,
            60.0
        )
    }

    override fun doInitialize()
    {
    }

    override fun dispose()
    {
        portal.close()
    }

    override fun isCompleted() = true
}
