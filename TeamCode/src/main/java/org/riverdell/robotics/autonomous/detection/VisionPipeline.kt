package org.riverdell.robotics.autonomous.detection

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.hardware

/**
 * Manages and configures all [VisionPortal] processors
 * for an op mode.
 */
class VisionPipeline(
    private val opMode: LinearOpMode
) : AbstractSubsystem()
{
    lateinit var portal: VisionPortal
    lateinit var sampleDetection: SampleDetection

    override fun start()
    {

    }

    override fun doInitialize()
    {
        sampleDetection = SampleDetection()
        portal = VisionPortal.Builder()
            .setCamera(
                opMode.hardware<WebcamName>("webcam")
            )
            .setCameraResolution(Size(640, 480))
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .addProcessors(sampleDetection)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        FtcDashboard.getInstance().startCameraStream(
            sampleDetection,
            30.0
        )
    }

    override fun dispose()
    {
        portal.close()
    }
}
