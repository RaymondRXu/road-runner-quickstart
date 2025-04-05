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
    private final Scalar lowerRed1 = new Scalar(0, 150, 50);   // Red lower bound (hue wrap-around)
    private final Scalar upperRed1 = new Scalar(10, 255, 255); // Red upper bound
    private final Scalar lowerRed2 = new Scalar(170, 150, 50); // Second range of red (for wrap-around hue)
    private final Scalar upperRed2 = new Scalar(180, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 50); // Blue lower bound
    private final Scalar upperBlue = new Scalar(130, 255, 255);// Blue upper bound

    private final Scalar lowerYellow = new Scalar(20, 150, 50); // Yellow lower bound
    private final Scalar upperYellow = new Scalar(30, 255, 255); // Yellow upper bound

    private Mat hsv = new Mat();
    private Mat maskRed = new Mat();
    private Mat maskBlue = new Mat();
    private Mat maskYellow = new Mat();
    private Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
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

        // Process red, blue, and yellow
        detectAndDrawRectangles(input, maskRed, new Scalar(255, 0, 0)); // Red bounding box
        detectAndDrawRectangles(input, maskBlue, new Scalar(0, 0, 255)); // Blue bounding box
        detectAndDrawRectangles(input, maskYellow, new Scalar(0, 255, 255)); // Yellow bounding box

        // Return the frame with rectangles and angles drawn
        return input;
    }

    /**
     * Detects the largest rectangle in the given mask, draws its bounding box on the frame, and displays its angle.
     * @param input The original frame.
     * @param mask The binary mask for a specific color.
     * @param boxColor The color for the bounding box (Scalar format: BGR).
     */

    @SuppressLint("DefaultLocale")
    private void detectAndDrawRectangles(Mat input, Mat mask, Scalar boxColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            // Find the largest contour
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
                // Find the minimum area bounding rectangle for the largest contour
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                // Draw the rectangle
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], boxColor, 2);
                }

                // Calculate the angle
                angle = rect.angle;
                center = rect.center;
                if (rect.size.width < rect.size.height) {
                    angle += 90;
                }
                if (angle < 0) {
                    angle += 180;
                }

                // Display the angle on the frame
                Imgproc.putText(input, "Angle: " + String.format("%.2f", angle), rect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            }
        }
    }
    public double returnAngle(){
        return angle;
    }
    public Point returnCenter() { return center; }
}