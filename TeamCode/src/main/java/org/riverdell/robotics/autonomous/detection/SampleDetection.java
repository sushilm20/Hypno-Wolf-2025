package org.riverdell.robotics.autonomous.detection;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

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

    /**
     * Turn Factor: These fields are responsible for tuning the PD controller
     * that rotates the end-effector toward the desired sample.
     *
     * TURN_FACTOR: The P parameter.
     * TURN_FACTOR_D_GAIN: The D parameter.
     */
    public static double TURN_FACTOR = 0.001;
    public static double TURN_FACTOR_D_GAIN = -0.0001;

    /**
     * Frame Center: Marks the center of the captured frame. The values
     * are typically your webcam width and height divided by 2.
     *
     * Currently tuned for a Logitech C270 Webcam.
     */
    public static double FRAME_CENTER_X = 640.0 / 2;
    public static double FRAME_CENTER_Y = 480.0 / 2;

    /**
     * Translational Guidance: Used to return a target Pose/Vector2D that
     * can be passed into a localizer system in order to lock the robot onto a specific sample.
     *
     * Not needed if you don't have a localizer.
     */
    public static double X_TRANSLATIONAL_GUIDANCE_SCALE = 0.05;
    public static double Y_TRANSLATIONAL_GUIDANCE_SCALE = 0.05;

    /**
     * Sample Area Average: Used to filter out any wrongly-identified samples by
     * ensuring each detection's bounding box area fits in the interval:
     *
     * [HOVER - DEVIATION, HOVER + DEVIATION]
     *
     * Tune this to your liking. Ensure you are tuning when the webcam
     * is in the position that it will pickup in.
     */
    public static double SAMPLE_AREA_AVERAGE_DURING_HOVER = 10000.0;
    public static double SAMPLE_AREA_AVERAGE_DURING_HOVER_MAX_DEVIATION = 4500.0;

    /**
     * Minimum Sample Area: Prevents any random outlier detections from
     * showing up and being considered in the autosnap feature.
     */
    public static double MIN_SAMPLE_AREA = 5000.0;

    /**
     * Sample Tracking Distance: A circle from the pixel radius from the
     * Frame Center in which samples centers must be in.
     */
    public static double SAMPLE_TRACKING_DISTANCE = 300.0;

    /**
     * AutoSnap: Keeps
     */
    public static double SAMPLE_AUTOSNAP_RADIUS = 25.0;
    public static double SAMPLE_AUTOSNAP_LIFETIME = 500L;

    /**
     * AutoTune: Automatically tunes the color ranges at a competition to
     * ensure selection accuracy.
     */
    public static int COLOR_AUTOTUNE_MODE = 0;

    // AtomicReference to store the last frame as a Bitmap
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private @Nullable Pose guidanceVector = null;

    private double previousRotationAngle = 0.0;
    private double guidanceRotationAngle = 0.0;
    private double sampleArea = 0.0;

    private Point autoSnapCenterLock = null;
    private long autoSnapHeartbeat = 0L;

    private SampleType detectionType = SampleType.BLUE;
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

    // New constants for color ranges (initially set to default values)
    private Scalar detectedColorRangeMin = new Scalar(0, 0, 0);
    private Scalar detectedColorRangeMax = new Scalar(255, 255, 255);

    // Method to automatically determine the color range based on the detected sample
    private void calculateColorRange(Mat input, Rect boundingBox) {
        // Crop the image to the bounding box area
        Mat croppedSample = new Mat(input, boundingBox);

        // Convert the cropped sample to the HSV color space
        Mat hsvSample = new Mat();
        Imgproc.cvtColor(croppedSample, hsvSample, Imgproc.COLOR_RGB2HSV);

        // Calculate the average color in the cropped area
        Scalar averageColor = Core.mean(hsvSample);

        // Set the detected color range based on the average color
        detectedColorRangeMin = new Scalar(
                Math.max(averageColor.val[0] - 10, 0),
                Math.max(averageColor.val[1] - 50, 0),
                Math.max(averageColor.val[2] - 50, 0)
        );
        detectedColorRangeMax = new Scalar(
                Math.min(averageColor.val[0] + 10, 180),
                Math.min(averageColor.val[1] + 50, 255),
                Math.min(averageColor.val[2] + 50, 255)
        );

        // Release the cropped sample and HSV Mat
        croppedSample.release();
        hsvSample.release();
    }

    // Modify the existing processFrame method to incorporate the new color detection logic
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create a mask for the specified color range
        Mat colorMask = new Mat();
        if (COLOR_AUTOTUNE_MODE == 0) {
            Core.inRange(
                    hsvMat,
                    detectedColorRangeMin,
                    detectedColorRangeMax,
                    colorMask
            );
        } else {
            Core.inRange(
                    hsvMat,
                    detectionType.getColorRangeMinimum(),
                    detectionType.getColorRangeMaximum(),
                    colorMask
            );
        }

        // Detect the sample object in the specified mask
        Point sampleCenter = detectSample(input, colorMask);
        Imgproc.circle(input, new Point(FRAME_CENTER_X, FRAME_CENTER_Y), (int) SAMPLE_TRACKING_DISTANCE, new Scalar(128, 0, 0));

        // If a sample is detected, calculate the color range
        if (sampleCenter != null) {
            Rect boundingBox = Imgproc.boundingRect(new MatOfPoint(sampleCenter)); // Get bounding box for the detected sample
            calculateColorRange(input, boundingBox); // Calculate the color range
            annotateBoundingBox(input, sampleCenter);
        }

        // Convert the processed frame (Mat) to a Bitmap for FTC dashboard display
        Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(COLOR_AUTOTUNE_MODE == 1 ? input : colorMask, bitmap);

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

        // Loop through contours to find the largest contour (which should be the sample)
        for (MatOfPoint contour : contours) {
            Rect localBoundingBox = Imgproc.boundingRect(contour);
            Point localSampleCenter = new Point((localBoundingBox.x + localBoundingBox.width / 2.0), (localBoundingBox.y + localBoundingBox.height / 2.0));
            double distance = Math.hypot(localSampleCenter.x - FRAME_CENTER_X, localSampleCenter.y - FRAME_CENTER_Y);
            double area = localBoundingBox.area();

            if (area < MIN_SAMPLE_AREA || distance > minDistance) {
                continue;
            }

            if (Math.abs(area - SAMPLE_AREA_AVERAGE_DURING_HOVER) < SAMPLE_AREA_AVERAGE_DURING_HOVER_MAX_DEVIATION) {
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

        return currentPose.add(guidanceVector);
    }

    public Pose calculateGuidanceVector(Point sampleCenter) {
        // Calculate the difference in pixels
        double xDiff = sampleCenter.x - FRAME_CENTER_X; // Positive means right, negative means left
        double yDiff = sampleCenter.y - FRAME_CENTER_Y; // Positive means down, negative means up

        // Convert pixel differences to applicable units
        double xMovement = xDiff * X_TRANSLATIONAL_GUIDANCE_SCALE;
        double yMovement = yDiff * Y_TRANSLATIONAL_GUIDANCE_SCALE;

        return new Pose(xMovement, yMovement); // Return guidance vector
    }

    private void annotateBoundingBox(Mat input, Point sampleCenter) {

        // Annotate the angle and servo position on the frame
        targetWristPosition = calculateServoPosition(currentWristPosition.get(), guidanceRotationAngle);

        Imgproc.putText(input, "Rotate: " + String.format("%.2f", guidanceRotationAngle) + " degrees", new Point(sampleCenter.x - 50, sampleCenter.y - 20),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Imgproc.putText(input, "Servo: " + String.format("%.2f", targetWristPosition), new Point(sampleCenter.x - 50, sampleCenter.y + 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 0, 0), 3);

        Pose guidance = guidanceVector = calculateGuidanceVector(sampleCenter);
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
        if (newServoPosition > 0.7) {
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
