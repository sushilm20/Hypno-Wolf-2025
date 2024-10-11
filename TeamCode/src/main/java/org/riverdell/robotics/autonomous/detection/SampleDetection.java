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

import javax.annotation.Nullable;

@Config
public class SampleDetection implements CameraStreamSource, VisionProcessor {

    public static int BLUE_LOW_B = 90;
    public static int BLUE_LOW_G = 100;
    public static int BLUE_LOW_R = 100;

    public static int BLUE_HIGH_B = 130;
    public static int BLUE_HIGH_G = 255;
    public static int BLUE_HIGH_R = 255;

    public static double TURN_FACTOR = 0.005; // Adjust to fine-tune servo movements
    public static double MAX_SERVO_POSITION = 1.0;
    public static double MIN_SERVO_POSITION = 0.0;

    public static double FRAME_CENTER_X = 1920.0 / 2;
    public static double FRAME_CENTER_Y = 1080.0 / 2;

    public static final double X_TRANSITIONAL_GUIDANCE_SCALE = 0.05;
    public static final double Y_TRANSITIONAL_GUIDANCE_SCALE = 0.05;

    private @Nullable Vector2d guidanceVector = null;

    // AtomicReference to store the last frame as a Bitmap
    private AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private double guidanceRotationAngle = 0.0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize the last frame with the correct size
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define color bounds for detecting the blue sample
        Scalar lowerBlueBound = new Scalar(BLUE_LOW_B, BLUE_LOW_G, BLUE_LOW_R);
        Scalar upperBlueBound = new Scalar(BLUE_HIGH_B, BLUE_HIGH_G, BLUE_HIGH_R);

        // Create a mask for the blue color range
        Mat blueMask = new Mat();
        Core.inRange(hsvMat, lowerBlueBound, upperBlueBound, blueMask);

        // Detect the sample object in the blue mask
        Point sampleCenter = detectSample(input, blueMask);

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
        double maxArea = -1.0;

        // Loop through contours to find the largest contour (which should be the sample)
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;

                // Calculate the bounding box of the sample
                boundingBox = Imgproc.boundingRect(contour);

                // Calculate the center point of the sample
                sampleCenter = new Point((boundingBox.x + boundingBox.width / 2.0), (boundingBox.y + boundingBox.height / 2.0));

                // Calculate the rotation angle of the sample using the minimum area bounding rectangle
                guidanceRotationAngle = calculateRotationAngle(contour);
            }
        }

        // Draw a rectangle around the detected sample
        if (boundingBox != null) {
            Imgproc.rectangle(input, boundingBox, new Scalar(117, 38, 191), 5);
        }

        return sampleCenter;
    }

    public @NotNull Pose getTargetPose(@NotNull Pose currentPose)
    {
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
        // Draw a circle at the center of the sample
        Imgproc.circle(input, sampleCenter, 10, new Scalar(251, 0, 255), 10);

        // Annotate the angle and servo position on the frame
        double servoPosition = calculateServoPosition(guidanceRotationAngle);

        Imgproc.putText(input, "Rotate: " + String.format("%.2f", guidanceRotationAngle) + " degrees", new Point(sampleCenter.x - 50, sampleCenter.y - 20),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(122, 193, 255), 3);

        Imgproc.putText(input, "Servo: " + String.format("%.2f", servoPosition), new Point(sampleCenter.x - 50, sampleCenter.y + 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(122, 193, 255), 3);

        Vector2d guidance = guidanceVector = calculateGuidanceVector(sampleCenter);
        Imgproc.putText(input, "Guidance: " + guidance, new Point(sampleCenter.x - 50, sampleCenter.y + 150),
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

    private double calculateServoPosition(double rotationAngle) {
        // Calculate the servo adjustment based on the rotation angle
        double servoAdjustment = rotationAngle * TURN_FACTOR;
        double currentServoPosition = 0.5; // Assuming the servo starts at a middle position (0.5)

        // Apply the adjustment to the current servo position
        double newServoPosition = currentServoPosition + servoAdjustment;

        // Ensure the servo position stays within valid bounds [0, 1]
        if (newServoPosition > MAX_SERVO_POSITION) {
            newServoPosition = MAX_SERVO_POSITION;
        } else if (newServoPosition < MIN_SERVO_POSITION) {
            newServoPosition = MIN_SERVO_POSITION;
        }

        return newServoPosition;
    }

    public double getGuidanceRotationAngle() {
        return guidanceRotationAngle;
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
