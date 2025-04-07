package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Test Sample Detector", group = "Vision")
public class TestSampleDetector extends LinearOpMode {
    private OpenCvCamera webcam;
    private SampleDetector sampleDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the camera monitor view ID to enable live preview on the Driver Station.
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Retrieve the webcam from the hardware map (ensure the configuration name matches).
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the webcam instance and set up the pipeline.
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sampleDetector = new SampleDetector(telemetry);
        webcam.setPipeline(sampleDetector);

        // Open the camera asynchronously.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming at a resolution of 320x240 in an upright orientation.
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

        while (opModeIsActive()) {
            // Display telemetry data provided by the SampleDetector pipeline.
            telemetry.addData("Distance", sampleDetector.getDistance());
            // If you add a public getter (e.g., getSampleAngle()) in SampleDetector,
            // you can also display the sample angle:
            // telemetry.addData("Sample Angle", sampleDetector.getSampleAngle());
            telemetry.update();
        }
    }
}
