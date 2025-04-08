package org.firstinspires.ftc.teamcode.opmodes.auto;

//roadrunner imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

//ftc imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

//vision
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;
import org.firstinspires.ftc.teamcode.vision.BlueAlignment;

@Autonomous(name = "Test Auto", group = "Autonomous")
public class VisionTestAuto extends LinearOpMode {
    public class BridgeArmClaw {
        private final Servo bridge;
        private final Servo sampleClaw;
        private final Servo specimenArm;
        private final Servo specimenClaw;
        private final Servo rail;
        private final ElapsedTime timer = new ElapsedTime();
        public BridgeArmClaw (HardwareMap hardwareMap){
            bridge = hardwareMap.get(Servo.class, "bridge");
            sampleClaw = hardwareMap.get(Servo.class, "sampleClaw");
            specimenArm = hardwareMap.get(Servo.class, "specimenArm");
            specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
            rail = hardwareMap.get(Servo.class, "rail");

            specimenClaw.setPosition(0); //start SpecimenClaw open
            specimenArm.setPosition(0);
            rail.setPosition(0);//rail set down
            bridge.setPosition(0.5);
        }

        public class CloseSpecimenClaw implements Action{
            private boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset){
                    isReset = true;
                    timer.reset();
                }
                specimenClaw.setPosition(0.9);//0.8
                return !(timer.seconds() > 0.5);
            }
        } //CloseSpecimenClaw

        public class ResetRailArm implements Action {
            private boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    timer.reset();
                    specimenArm.setPosition(0); //set to reset value (back)
                    rail.setPosition(0.5); //set to reset value (down)
                    isReset = true;
                }
                if (timer.seconds() > 0.8){
                    isReset = false;
                    return false; //exit after reset
                }
                return true;
            }
        } //ResetRailArm

        public class ResetTest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenArm.setPosition(1);//set to reset value (back)
                rail.setPosition(0.5); //set to reset value (down)
                return false;
            }
        }

        public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimenClaw.setPosition(0);
                return false;
            }
        }

        public class openSamClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sampleClaw.setPosition(0);
                return false;
            }
        }

        public class closeSamClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bridge.setPosition(0.96);
                sampleClaw.setPosition(0.8);
                return true;
            }
        }

        public class hoverArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bridge.setPosition(0.85);
                return true;
            }
        }

        public class raiseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bridge.setPosition(0.5);
                return true;
            }
        }

        public class SleepAction implements Action {
            private final long sleepTimeMillis; // Duration to sleep in milliseconds
            private long startTime; // When the sleep started

            public SleepAction(long sleepTimeMillis) {
                this.sleepTimeMillis = sleepTimeMillis;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (startTime == 0) {
                    // Start the timer on the first run
                    startTime = System.currentTimeMillis();
                }

                // Check if the sleep duration has passed
                long elapsedTime = System.currentTimeMillis() - startTime;
                return elapsedTime < sleepTimeMillis; // Sleep is done
// Sleep is still ongoing
            }
        }


        //This action will be used to hang specimen in the sequential actions
        public class HangSpecimen implements Action {
            private boolean isReset = false;
            @Override
            public boolean run (@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    isReset = true;
                    timer.reset();
                }
                /*rail should go up, swing specimenArm to position
                rail => 0.5; sA => 0*/
                rail.setPosition(0.57); //0.61
                specimenArm.setPosition(1);
                telemetry.addData("rail", rail.getPosition());
                //change these positions if needed

                if (timer.seconds() >= 1.5) {
                    isReset = false;
                    specimenClaw.setPosition(0); //open SpecimenClaw
                    return false;
                }
                return true; //false exits
            }
        } //hangSpecimen

        //Declare Actions
        public Action closeSpecimenClaw() { return new CloseSpecimenClaw(); }
        // public Action rrTest(){ return new ResetTest(); }
        public Action openClaw(){ return new openClaw(); }
        public Action openSamClaw() { return new openSamClaw(); }
        public Action closeSamClaw() { return new closeSamClaw(); }
        public Action sleep(int a) { return new SleepAction(a); }
        public Action hangSpecimen() { return new HangSpecimen(); }
        public Action hoverArm() { return new hoverArm(); }
        public Action raiseArm() { return new raiseArm(); }
        public Action resetRailArm(){ return new ResetRailArm(); }
    } //BridgeArmClaw

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing vision
        OpenCvCamera webcam;
        BlueAlignment blueAlignment;

        int camMonitorId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(camName, camMonitorId);
        blueAlignment = new BlueAlignment();
        webcam.setPipeline(blueAlignment);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("CV Error", errorCode);
                telemetry.update();
            }
        });


        Pose2d initialPose = new Pose2d(25, -62, Math.toRadians(90));

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if ((robotPose.position.x.value() < 18.0) && (robotPose.position.y.value() > -48))  {
                return 40.0;
            } else {
                return 80.0;
            }
        };

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose); //initialize drive
        BridgeArmClaw bac = new BridgeArmClaw(hardwareMap); //initialize all servos

        // VISION CENTERING
        final double imageCenterX = 160;
        final double imageCenterY = 120;
        final double threshold = 20;
        Pose2d currentVisionPose = new Pose2d(25,-62,Math.toRadians(90));
        // Conversion factor from pixels to inches.
        final double conversionFactor = 0.1;
        boolean centered = false;
        Point sampleCenter = blueAlignment.returnCenter();
        double dx = sampleCenter.x - imageCenterX;
        double dy = sampleCenter.y - imageCenterY;
        // Only consider downward movement (ignore negative vertical error).
        double adjustedDy = (dy > 0) ? dy : 0;
        // Calculate full correction in inches.
        double moveX = dx * conversionFactor;
        double moveY = adjustedDy * conversionFactor;
        Vector2d targetVector = new Vector2d(currentVisionPose.position.x + moveX, currentVisionPose.position.y + moveY);

        // TRAJECTORIES
        // HANG PRELOADED SPECIMEN
        TrajectoryActionBuilder spec0Traj = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-3,-31), baseVelConstraint);
        // MOVE TO ALIGN NEW SAMPLE
        TrajectoryActionBuilder alignSample = spec0Traj.endTrajectory().fresh()
                .strafeToConstantHeading(targetVector);
        // DEPOSIT NEW SAMPLE
        TrajectoryActionBuilder depositSample = spec0Traj.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(25, -62, Math.toRadians(0)), Math.toRadians(0));
        // PUSH SAMPLES
        TrajectoryActionBuilder pushSamplesTraj = depositSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(44, -10, Math.toRadians(90)), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(48, -10))
                .strafeToConstantHeading(new Vector2d(48,-46))
                .splineToConstantHeading(new Vector2d(56,-13), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(56,-46))
                .splineToConstantHeading(new Vector2d(61, -13), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(61, -46))
                .strafeToConstantHeading(new Vector2d(38, -59));
        // HANG FIRST SPECIMEN
        TrajectoryActionBuilder specimen1Traj = pushSamplesTraj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-3, -31), baseVelConstraint);
        // TAKE SECOND SPECIMEN
        TrajectoryActionBuilder specimen2TrajBack = specimen1Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        // HANG SECOND SPECIMEN
        TrajectoryActionBuilder specimen2TrajHang = specimen2TrajBack.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2.5, -31), baseVelConstraint);
        // TAKE THIRD SPECIMEN
        TrajectoryActionBuilder specimen3TrajBack = specimen2TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        // HANG THIRD SPECIMEN
        TrajectoryActionBuilder specimen3TrajHang = specimen3TrajBack.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2, -31), baseVelConstraint);
        // TAKE FOURTH SPECIMEN
        TrajectoryActionBuilder specimen4TrajBack = specimen3TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        // HANG FOURTH SPECIMEN
        TrajectoryActionBuilder specimen4TrajHang = specimen4TrajBack.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-1.5, -31), baseVelConstraint);
        // TAKE FIFTH SPECIMEN
        TrajectoryActionBuilder specimen5TrajBack = specimen4TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        // HANG FIFTH SPECIMEN
        TrajectoryActionBuilder specimen5TrajHang = specimen5TrajBack.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-1, -31), baseVelConstraint);
        // PARK
        TrajectoryActionBuilder parkTraj = specimen5TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        waitForStart(); //wait until start is pressed
        if (isStopRequested()) return; // exit when stop is pressed

        // ACTIONS
        // SCAN SAMPLES (SPEC 0)
        SequentialAction scanSamples = new SequentialAction(
                new ParallelAction(
                        spec0Traj.build(),
                        bac.hangSpecimen(),
                        bac.hoverArm()
                ),
                new SequentialAction(
                        bac.resetRailArm(),
                        alignSample.build(),
                        bac.closeSamClaw(),
                        bac.raiseArm(),
                        depositSample.build(),
                        bac.openSamClaw()
                )
        );
        // PUSH SAMPLES
        SequentialAction pushSamples = new SequentialAction(
                new ParallelAction(
                        pushSamplesTraj.build()
                )
        );
        // HANG FIRST SPECIMEN
        SequentialAction hangSpecimen1 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction(
                        specimen1Traj.build(),
                        bac.hangSpecimen()
                )
        );
        // HANG SECOND SPECIMEN
        SequentialAction hangSpecimen2 = new SequentialAction(
                new ParallelAction(
                        specimen2TrajBack.build(),
                        bac.resetRailArm(),
                        bac.openClaw()
                ),
                new SequentialAction(
                        bac.sleep(100),
                        bac.closeSpecimenClaw()
                ),
                new ParallelAction(
                        specimen2TrajHang.build(),
                        bac.hangSpecimen()
                ),
                bac.openClaw()
        );
        // HANG THIRD SPECIMEN
        SequentialAction hangSpecimen3 = new SequentialAction(
                new ParallelAction(
                        specimen3TrajBack.build(),
                        bac.resetRailArm(),
                        bac.openClaw()
                ),
                new SequentialAction(
                        bac.sleep(100),
                        bac.closeSpecimenClaw()
                ),
                new ParallelAction(
                        specimen3TrajHang.build(),
                        bac.hangSpecimen()
                ),
                bac.openClaw()
        );
        // HANG FOURTH SPECIMEN
        SequentialAction hangSpecimen4 = new SequentialAction(
                new ParallelAction(
                        specimen4TrajBack.build(),
                        bac.resetRailArm(),
                        bac.openClaw()
                ),
                new SequentialAction(
                        bac.sleep(100),
                        bac.closeSpecimenClaw()
                ),
                new ParallelAction(
                        specimen4TrajHang.build(),
                        bac.hangSpecimen()
                ),
                bac.openClaw()
        );
        // HANG FIFTH SPECIMEN
        SequentialAction hangSpecimen5 = new SequentialAction(
                new ParallelAction(
                        specimen5TrajBack.build(),
                        bac.resetRailArm(),
                        bac.openClaw()
                ),
                new SequentialAction(
                        bac.sleep(100),
                        bac.closeSpecimenClaw()
                ),
                new ParallelAction(
                        specimen5TrajHang.build(),
                        bac.hangSpecimen()
                ),
                bac.openClaw()
        );
        // PARK
        SequentialAction park = new SequentialAction(
                new ParallelAction(
                        bac.resetRailArm(),
                        parkTraj.build()
                )
        );
        // RUN ACTIONS
        Actions.runBlocking(
                new SequentialAction(
                        scanSamples,
                        pushSamples,
                        hangSpecimen1,
                        hangSpecimen2,
                        hangSpecimen3,
                        hangSpecimen4,
                        hangSpecimen5,
                        park
                )
        );

        telemetry.update();
    }
}