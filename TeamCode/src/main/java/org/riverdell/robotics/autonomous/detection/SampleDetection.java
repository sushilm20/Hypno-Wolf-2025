package org.riverdell.robotics.autonomous.detection;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import javax.annotation.Nullable;

@Config
public class SampleDetection implements CameraStreamSource, VisionProcessor {

    public static double TURN_FACTOR = 0.001;
    public static double TURN_FACTOR_D_GAIN = -0.0001;

    public static double FRAME_CENTER_X = 640.0 / 2;
    public static double FRAME_CENTER_Y = 480.0 / 2;

    public static double X_TRANSITIONAL_GUIDANCE_SCALE = 0.05;
    public static double Y_TRANSITIONAL_GUIDANCE_SCALE = 0.05;

    public static double MIN_SAMPLE_AREA = 5000.0;
    public static double SAMPLE_AREA_ROLLING_AVERAGE_MAX_DEVIATION = 4500.0;

    public static double SAMPLE_TRACKING_DISTANCE = 300.0;

    public static double SAMPLE_AUTOSNAP_RADIUS = 25.0;
    public static double SAMPLE_AUTOSNAP_LIFETIME = 500L;

    // AtomicReference to store the last frame as a Bitmap
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private @Nullable Vector2d guidanceVector = null;

    private double previousRotationAngle = 0.0;
    private double guidanceRotationAngle = 0.0;
    private double sampleArea = 0.0;

    private final RollingAverage areaAverage = new RollingAverage(10);
    private boolean areaAverageDirty = false;

    private Point autoSnapCenterLock = null;
    private long autoSnapHeartbeat = 0L;

    public void markAreaAverageDirty() {
        areaAverageDirty = true;
    }

    private SampleType detectionType = SampleType.Blue;
    private Supplier<Double> currentWristPosition = () -> 0.0;

    private double targetWristPosition = 0.0;

    public double getTargetWristPosition() {
        return targetWristPosition;
    }

    public void supplyCurrentWristPosition(Supplier<Double> currentWristPosition) {
        this.currentWristPosition = currentWristPosition;
    }

    public void setDetectionType(SampleType sampleType) {
        this.detectionType = sampleType;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize the last frame with the correct size
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create a mask for the specified color range
        Mat colorMask = new Mat();
        Core.inRange(
                hsvMat,
                detectionType.getColorRangeMinimum(),
                detectionType.getColorRangeMaximum(),
                colorMask
        );

        // Detect the sample object in the specified mask
        Point sampleCenter = detectSample(input, colorMask);
        Imgproc.circle(input, new Point(FRAME_CENTER_X, FRAME_CENTER_Y), (int) SAMPLE_TRACKING_DISTANCE, new Scalar(128, 0, 0));

        // Annotate the detected sample with bounding box and angle
        if (sampleCenter != null) {
            annotateBoundingBox(input, sampleCenter);
        }

        // Convert the processed frame (Mat) to a Bitmap for FTC dashboard display
        Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bitmap);

        // Update the last frame
        lastFrame.set(bitmap);

        return input;
    }

    private Point detectSample(Mat input, Mat blueMask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect boundingBox = null;
        Point sampleCenter = null;
        double minDistance = SAMPLE_TRACKING_DISTANCE;

        if (areaAverageDirty) {
            areaAverage.reset();
            areaAverageDirty = false;
        }

        // Loop through contours to find the largest contour (which should be the sample)
        for (MatOfPoint contour : contours) {
            Rect localBoundingBox = Imgproc.boundingRect(contour);
            Point localSampleCenter = new Point((localBoundingBox.x + localBoundingBox.width / 2.0), (localBoundingBox.y + localBoundingBox.height / 2.0));
            double distance = Math.hypot(localSampleCenter.x - FRAME_CENTER_X, localSampleCenter.y - FRAME_CENTER_Y);
            double area = localBoundingBox.area();

            if (area < MIN_SAMPLE_AREA || distance > minDistance) {
                continue;
            }

            boolean outlier = areaAverage.isOutlier(area, SAMPLE_AREA_ROLLING_AVERAGE_MAX_DEVIATION);
            if (!areaAverage.isAvailable() || !outlier) {
                areaAverage.add(area);
            }

            if (areaAverage.isAvailable() && !outlier) {
                boundingBox = localBoundingBox;
                sampleCenter = localSampleCenter;
                minDistance = distance;
                sampleArea = localBoundingBox.area();

                if (autoSnapCenterLock != null) {
                    if (Math.hypot(localSampleCenter.x - autoSnapCenterLock.x, localSampleCenter.y - autoSnapCenterLock.y) < SAMPLE_AUTOSNAP_RADIUS) {
                        Imgproc.circle(input, localSampleCenter, (int) SAMPLE_AUTOSNAP_RADIUS, new Scalar(0, 128, 0));

                        autoSnapHeartbeat = System.currentTimeMillis();
                        autoSnapCenterLock = localSampleCenter;
                        break;
                    }

                    if (System.currentTimeMillis() - autoSnapHeartbeat < SAMPLE_AUTOSNAP_LIFETIME) {
                        return autoSnapCenterLock;
                    }
                }

                guidanceRotationAngle = calculateRotationAngle(contour);
            }

            // add a rectangle showing we detected this sample
            Imgproc.rectangle(input, localBoundingBox, new Scalar(117, 38, 2), 1);
        }

        // Draw a rectangle around the detected sample
        if (boundingBox != null) {
            if (autoSnapCenterLock == null) {
                autoSnapCenterLock = sampleCenter;
                autoSnapHeartbeat = System.currentTimeMillis();
            }

            Imgproc.rectangle(input, boundingBox, new Scalar(117, 38, 191), 5);
        }

        return sampleCenter;
    }

    public @NotNull Pose getTargetPose(@NotNull Pose currentPose) {
        if (guidanceVector == null) {
            return currentPose;
        }

        return currentPose.addOnlyTranslational(guidanceVector);
    }

    public Vector2d calculateGuidanceVector(Point sampleCenter) {
        // Calculate the difference in pixels
        double xDiff = sampleCenter.x - FRAME_CENTER_X; // Positive means right, negative means left
        double yDiff = sampleCenter.y - FRAME_CENTER_Y; // Positive means down, negative means up

        // Convert pixel differences to applicable units
        double xMovement = xDiff * X_TRANSITIONAL_GUIDANCE_SCALE;
        double yMovement = yDiff * Y_TRANSITIONAL_GUIDANCE_SCALE;

        return new Vector2d(xMovement, yMovement); // Return guidance vector
    }

    private void annotateBoundingBox(Mat input, Point sampleCenter) {

        // Annotate the angle and servo position on the frame
        targetWristPosition = calculateServoPosition(currentWristPosition.get(), guidanceRotationAngle);

        Imgproc.putText(input, "Rotate: " + String.format("%.2f", guidanceRotationAngle) + " degrees", new Point(sampleCenter.x - 50, sampleCenter.y - 20),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Imgproc.putText(input, "Servo: " + String.format("%.2f", targetWristPosition), new Point(sampleCenter.x - 50, sampleCenter.y + 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Vector2d guidance = guidanceVector = calculateGuidanceVector(sampleCenter);
        Imgproc.putText(input, "Area: " + sampleArea, new Point(sampleCenter.x - 50, sampleCenter.y + 150),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);
    }

    private double calculateRotationAngle(MatOfPoint contour) {
        // Convert contour to MatOfPoint2f for the minAreaRect function
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Get the minimum area bounding rectangle for the contour
        RotatedRect rotRect = Imgproc.minAreaRect(contour2f);

        // Get the angle of the bounding rectangle
        double angle = rotRect.angle;

        // If width < height, the object is closer to vertical and the angle needs to be adjusted to correct to vertical orientation
        if (rotRect.size.width > rotRect.size.height) {
            angle += 90.0;
        }

        // Normalize the angle to [-90, 90] to get the smallest rotation required
        if (angle > 90) {
            angle -= 180.0;
        }

        return angle;
    }

    private double calculateServoPosition(double current, double rotationAngle) {
        // Calculate the derivative (change) in rotation angle
        double derivative = rotationAngle - previousRotationAngle;

        // Store the current angle for the next calculation
        previousRotationAngle = rotationAngle;

        // Calculate the servo adjustment based on the P and D terms
        double servoAdjustment = (rotationAngle * TURN_FACTOR) + (derivative * TURN_FACTOR_D_GAIN);

        // Apply the adjustment to the current servo position
        double newServoPosition = current + servoAdjustment;

        // Ensure the servo position stays within valid bounds [0, 1]
        if (newServoPosition > 1.0) {
            newServoPosition = 1.0;
        } else if (newServoPosition < 0.0) {
            newServoPosition = 0.0;
        }

        return newServoPosition;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> {
            // Pass the last frame (Bitmap) to the consumer
            bitmapConsumer.accept(lastFrame.get());
        });
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Optional: If you want to draw additional information on the canvas
    }
}
