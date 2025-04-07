package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleDetector;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;

@TeleOp(name = "Test Sample Vision Teleop", group = "Teleop")
public class TestSampleVisionTeleop extends LinearOpMode {
    private OpenCvCamera webcam;
    private SampleDetector sampleDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the webcam from the hardware map.
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Get the camera monitor view ID so the live feed appears on the Driver Station.
        int camMonitorId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Create the webcam instance using EasyOpenCV.
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, camMonitorId);

        // Initialize the SampleDetector pipeline (which you already have).
        sampleDetector = new SampleDetector(telemetry);
        webcam.setPipeline(sampleDetector);

        // Open the camera asynchronously.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming at 320x240 with UPRIGHT orientation.
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        // Loop and update telemetry with vision data.
        while (opModeIsActive()) {
            ArrayList<RotatedRect> rects = sampleDetector.filteredRects;
            if (rects == null || rects.isEmpty()) {
                telemetry.addData("Sample", "None detected");
            } else {
                // Assume the image center is at (320,240).
                double imageCenterX = 320;
                double imageCenterY = 240;
                double minDist = Double.MAX_VALUE;
                RotatedRect closestRect = null;
                for (RotatedRect rect : rects) {
                    Point center = rect.center;
                    double dx = center.x - imageCenterX;
                    double dy = center.y - imageCenterY;
                    double dist = Math.hypot(dx, dy);
                    if (dist < minDist) {
                        minDist = dist;
                        closestRect = rect;
                    }
                }
                if (closestRect != null) {
                    // Calculate the offset from center.
                    Point sampleCenter = closestRect.center;
                    double dx = sampleCenter.x - imageCenterX;
                    double dy = sampleCenter.y - imageCenterY;

                    // Decide suggested movement direction.
                    String direction;
                    if (Math.abs(dx) > Math.abs(dy)) {
                        direction = (dx < 0) ? "move left" : "move right";
                    } else {
                        direction = (dy < 0) ? "move up" : "move down";
                    }

                    // Get the sample's color (from your pipeline's static enum) and angle.
                    String color = SampleDetector.colorType.toString();
                    double angle = sampleDetector.realAngle();

                    telemetry.addData("Closest Sample", "Color: " + color + ", Angle: " + angle + "°");
                    telemetry.addData("Offset (dx,dy)", String.format("(%.1f, %.1f)", dx, dy));
                    telemetry.addData("Suggested Move", direction);
                }
            }
            telemetry.update();
        }
    }
}
