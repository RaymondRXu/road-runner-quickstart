package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.vision.Alignment;

@TeleOp(name = "Test Alignment Pipeline", group = "Vision")
public class TestAlignmentOpMode extends LinearOpMode {
    private OpenCvCamera webcam;
    private Alignment alignmentPipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        alignmentPipeline = new Alignment();

        webcam.setPipeline(alignmentPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error opening camera", errorCode);
            }
        });

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Optionally display angle and center data
            telemetry.addData("Angle", alignmentPipeline.returnAngle());
            telemetry.addData("Center", alignmentPipeline.returnCenter());
            telemetry.update();
        }
    }
}
