package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;

@TeleOp(name = "PLEASE DO NOT USE ME", group = "Vision")
public class TestAlignmentBlueOpMode extends LinearOpMode {
    private OpenCvCamera webcam;
    private BlueAlignment blueAlignment;

    @Override
    public void runOpMode() {
        // Obtain the view ID for live preview.
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the webcam instance.
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        blueAlignment = new BlueAlignment();
        webcam.setPipeline(blueAlignment);

        // Open the camera device asynchronously.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
        telemetry.addData("Note", "Use DS Camera Preview if desired");
        telemetry.update();
        waitForStart();

        final double imageCenterX = 160;
        final double imageCenterY = 120;
        final double threshold = 20;

        while (opModeIsActive()) {
            // Calculate offset from the assumed image center.
            Point sampleCenter = blueAlignment.returnCenter();
            double dx = sampleCenter.x - imageCenterX;
            double dy = sampleCenter.y - imageCenterY;

            // For vertical offset, ignore upward movement: if dy is negative, treat it as zero.
            double adjustedDy = (dy < threshold) ? 0 : dy;

            String direction;
            if (Math.abs(dx) < threshold && adjustedDy < threshold) {
                direction = "CENTERED";
            } else {
                String horiz = "";
                String vert = "";
                if (Math.abs(dx) >= threshold) {
                    horiz = (dx < 0) ? "move left by " + String.format("%.1f", -dx) + " pixels"
                            : "move right by " + String.format("%.1f", dx) + " pixels";
                }
                if (adjustedDy >= threshold) {
                    vert = "move down by " + String.format("%.1f", adjustedDy) + " pixels";
                }
                if (!horiz.isEmpty() && !vert.isEmpty())
                    direction = horiz + " and " + vert;
                else
                    direction = horiz + vert;
            }

            String color = blueAlignment.returnColor();
            double angle = blueAlignment.returnAngle();
            // Compute rotation: true if the angle is <= 40° or >= 140° (approximately horizontal).
            boolean rotation = (angle <= 40 || angle >= 140);

            telemetry.addData("Detected Sample", "Color: " + color + ", Angle: " + angle + "°");
            telemetry.addData("Offset (dx, dy)", String.format("(%.1f, %.1f)", dx, dy));
            telemetry.addData("Suggested Move", direction);
            telemetry.addData("Rotation (horizontal)", rotation);
            telemetry.update();
        }
    }
}
