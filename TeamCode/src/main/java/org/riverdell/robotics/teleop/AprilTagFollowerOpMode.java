package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;

@TeleOp(name = "AprilTag Detection Dashboard", group = "FTC")
public class AprilTagDetectionOpMode extends LinearOpMode {

    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Camera calibration values (adjust based on your webcam specs)
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166; // meters

    @Override
    public void runOpMode() {
        // Initialize Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
            TelemetryPacket packet = new TelemetryPacket();

            if (detections.size() == 0) {
                packet.put("Detected", "None");
            } else {
                for (AprilTagDetection tag : detections) {
                    packet.put("Tag ID", tag.id);
                    packet.put("Translation X (m)", tag.pose.x);
                    packet.put("Translation Y (m)", tag.pose.y);
                    packet.put("Translation Z (m)", tag.pose.z);
                }
            }

            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}
