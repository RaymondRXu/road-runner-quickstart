package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;

@TeleOp(name = "Test Alignment Pipeline", group = "Vision")
public class TestAlignmentOpMode extends LinearOpMode {
    private OpenCvCamera webcam;
    private Alignment alignmentPipeline;

    @Override
    public void runOpMode() {
        // Obtain the view ID for the camera monitor (live preview on the RC screen)
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the webcam instance with live preview enabled
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        alignmentPipeline = new Alignment();
        webcam.setPipeline(alignmentPipeline);

        // Open the camera device asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming locally at 320x240 with the correct orientation
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                // Also stream to FTC Dashboard at 30 FPS
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening camera", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Waiting for start");
        telemetry.addData("Dashboard", "Visit http://<RC_IP>:8080 to view the stream");
        telemetry.addData("Note", "Use DS Camera Preview (overflow menu) if desired");
        telemetry.update();
        waitForStart();

        // Assume the image center is at (320,240)
        final double imageCenterX = 160;
        final double imageCenterY = 120;
        // Increased threshold for "centered" to 20 pixels
        final double threshold = 20;

        while (opModeIsActive()) {
            // Display basic alignment data
            telemetry.addData("Angle", alignmentPipeline.returnAngle());
            telemetry.addData("Center", alignmentPipeline.returnCenter());

            // Calculate offset from the assumed image center.
            Point sampleCenter = alignmentPipeline.returnCenter();
            double dx = sampleCenter.x - imageCenterX;
            double dy = sampleCenter.y - imageCenterY;

            String direction;
            if (Math.abs(dx) < threshold && Math.abs(dy) < threshold) {
                direction = "CENTERED";
            } else {
                String horiz = "";
                String vert = "";
                if (Math.abs(dx) >= threshold) {
                    horiz = (dx < 0) ? "move left by " + String.format("%.1f", -dx) + " pixels"
                            : "move right by " + String.format("%.1f", dx) + " pixels";
                }
                if (Math.abs(dy) >= threshold) {
                    vert = (dy < 0) ? "move up by " + String.format("%.1f", -dy) + " pixels"
                            : "move down by " + String.format("%.1f", dy) + " pixels";
                }
                if (!horiz.isEmpty() && !vert.isEmpty()) {
                    direction = horiz + " and " + vert;
                } else {
                    direction = horiz + vert;
                }
            }

            // Get the sample's color and angle.
            String color = alignmentPipeline.returnColor();
            double angle = alignmentPipeline.returnAngle();

            telemetry.addData("Closest Sample", "Color: " + color + ", Angle: " + angle + "°");
            telemetry.addData("Offset (dx, dy)", String.format("(%.1f, %.1f)", dx, dy));
            telemetry.addData("Suggested Move", direction);
            telemetry.update();
        }
    }
}
