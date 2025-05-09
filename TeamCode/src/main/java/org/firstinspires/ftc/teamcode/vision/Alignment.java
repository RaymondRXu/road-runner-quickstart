package org.firstinspires.ftc.teamcode.vision;

import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Alignment extends OpenCvPipeline {
    double angle;
    Point center = new Point(0, 0);

    // Lower/Upper Color Bounds
    private final Scalar lowerRed1 = new Scalar(0, 150, 50);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);
    private final Scalar lowerRed2 = new Scalar(170, 150, 50);
    private final Scalar upperRed2 = new Scalar(180, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130, 255, 255);

    private final Scalar lowerYellow = new Scalar(20, 150, 50);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    private Mat hsv = new Mat();
    private Mat maskRed = new Mat();
    private Mat maskBlue = new Mat();
    private Mat maskYellow = new Mat();
    private Mat hierarchy = new Mat();

    // New fields for color detection
    private double maxDetectedArea = 0;
    private String sampleColor = "None";

    @Override
    public Mat processFrame(Mat input) {
        // Reset global detection fields each frame
        maxDetectedArea = 0;
        sampleColor = "None";

        // Convert the input frame to HSV color space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Red color masking (combine two masks for the hue wrap-around)
        Core.inRange(hsv, lowerRed1, upperRed1, maskRed);
        Mat maskRed2 = new Mat();
        Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
        Core.bitwise_or(maskRed, maskRed2, maskRed);

        // Blue color masking
        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

        // Yellow color masking
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

        // Process red, blue, and yellow detections
        detectAndDrawRectangles(input, maskRed, new Scalar(255, 0, 0));   // Red bounding box
        detectAndDrawRectangles(input, maskBlue, new Scalar(0, 0, 255));    // Blue bounding box
        detectAndDrawRectangles(input, maskYellow, new Scalar(0, 255, 255)); // Yellow bounding box

        return input;
    }

    @SuppressLint("DefaultLocale")
    private void detectAndDrawRectangles(Mat input, Mat mask, Scalar boxColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            double maxArea = 5000;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], boxColor, 2);
                }

                angle = rect.angle;
                center = rect.center;
                if (rect.size.width < rect.size.height) {
                    angle += 90;
                }
                if (angle < 0) {
                    angle += 180;
                }
                Imgproc.putText(input, "Angle: " + String.format("%.2f", angle), rect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

                // Update sampleColor if this rectangle's area is the largest seen so far in this frame.
                double area = rect.size.area();
                if (area > maxDetectedArea) {
                    maxDetectedArea = area;
                    if (boxColor.val[0] == 255 && boxColor.val[1] == 0 && boxColor.val[2] == 0)
                        sampleColor = "Red";
                    else if (boxColor.val[0] == 0 && boxColor.val[1] == 0 && boxColor.val[2] == 255)
                        sampleColor = "Blue";
                    else if (boxColor.val[0] == 0 && boxColor.val[1] == 255 && boxColor.val[2] == 255)
                        sampleColor = "Yellow";
                }
            }
        }
    }

    public double returnAngle() {
        return angle;
    }

    public Point returnCenter() {
        return center;
    }

    // New public method to return the detected sample color.
    public String returnColor() {
        return sampleColor;
    }
}
