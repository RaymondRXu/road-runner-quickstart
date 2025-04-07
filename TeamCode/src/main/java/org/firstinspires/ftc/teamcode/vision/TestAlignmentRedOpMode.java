package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;

@TeleOp(name = "Test Alignment Pipeline - Red", group = "Vision")
public class TestAlignmentRedOpMode extends LinearOpMode {
    private OpenCvCamera webcam;
    private Alignment alignmentPipeline;

    @Override
    public void runOpMode() {
        // Obtain the view ID for live preview.
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the webcam instance.
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        alignmentPipeline = new Alignment();
        webcam.setPipeline(alignmentPipeline);

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
            // Get basic alignment data.
            Point sampleCenter = alignmentPipeline.returnCenter();
            double dx = sampleCenter.x - imageCenterX;
            double dy = sampleCenter.y - imageCenterY;

            // Only process if the sample is below the image center.
            if (sampleCenter.y < imageCenterY) {
                telemetry.addData("Status", "Red sample detected above center; ignoring");
            } else if (!alignmentPipeline.returnColor().equalsIgnoreCase("Red")) {
                telemetry.addData("Status", "No RED sample detected");
            } else {
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
                String color = alignmentPipeline.returnColor();
                double angle = alignmentPipeline.returnAngle();
                telemetry.addData("Detected Sample", "Color: " + color + ", Angle: " + angle + "°");
                telemetry.addData("Offset (dx, dy)", String.format("(%.1f, %.1f)", dx, dy));
                telemetry.addData("Suggested Move", direction);
            }
            telemetry.update();
        }
    }
}
