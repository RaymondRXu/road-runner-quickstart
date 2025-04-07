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

public class BlueAlignment extends OpenCvPipeline {
    double angle;
    Point center = new Point(0, 0);

    // Blue color bounds
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130, 255, 255);

    private Mat hsv = new Mat();
    private Mat maskBlue = new Mat();
    private Mat hierarchy = new Mat();

    // For tracking the largest blue detection.
    private double maxDetectedArea = 0;
    private String sampleColor = "None";

    @Override
    public Mat processFrame(Mat input) {
        // Reset detection fields
        maxDetectedArea = 0;
        sampleColor = "None";
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Blue masking only.
        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

        // Process blue detections.
        detectAndDrawRectangles(input, maskBlue, new Scalar(0, 0, 255)); // Blue bounding box

        return input;
    }

    @SuppressLint("DefaultLocale")
    private void detectAndDrawRectangles(Mat input, Mat mask, Scalar boxColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            double minAreaThreshold = 5000;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > minAreaThreshold) {
                    largestContour = contour;
                    minAreaThreshold = area; // choose the largest contour.
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

                double area = rect.size.area();
                if (area > maxDetectedArea) {
                    maxDetectedArea = area;
                    sampleColor = "Blue";
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
    public String returnColor() {
        return sampleColor;
    }
}
