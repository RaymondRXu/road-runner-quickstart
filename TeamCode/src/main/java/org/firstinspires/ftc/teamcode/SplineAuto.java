package org.firstinspires.ftc.teamcode;

//roadrunner imports
import androidx.annotation.NonNull;

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
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Spline Auto", group = "Autonomous")
public class SplineAuto extends LinearOpMode {
    public class BridgeArmClaw {
        private Servo bridge, sampleClaw, specimenArm, specimenClaw, rail;
        private ElapsedTime timer = new ElapsedTime();
        //multi declaration of Servo variables

        //Pre Cond: initialize the Servo Vars through HardwareMap
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
        }//Constructor

        public class CloseSpecimenClaw implements Action{
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset){
                    isReset = true;
                    timer.reset();
                }
                specimenClaw.setPosition(0.9);//0.8
                if (timer.seconds() > 0.5) return false;
                return true;
            }
        }//CloseSpecimenArm

        public class ResetRailArm implements Action {
            private boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    timer.reset();
                    specimenArm.setPosition(0);//set to reset value (back)
                    rail.setPosition(0.5); //set to reset value (down)

                    isReset = true;
                }

                if (timer.seconds() > 0.8){

                    isReset = false;
                    return false;//exit after reset
                }

                return true;
            }
        }//ResetRailArm

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
                if (elapsedTime >= sleepTimeMillis)
                    return false; // Sleep is done
                return true; // Sleep is still ongoing
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
        }//hangSpecimen

        //Declare Actions

        public Action closeSpecimenClaw(){
            return new CloseSpecimenClaw();
        }

        public Action rrTest(){
            return new ResetTest();
        }

        public Action openClaw(){
            return new openClaw();
        }

        public Action sleep(int a) {
            return new SleepAction(a);
        }

        public Action hangSpecimen() {
            return new HangSpecimen();
        }

        public Action resetRailArm(){
            return new ResetRailArm();
        }


    }//BridgeArmClaw

    @Override
    public void runOpMode() throws InterruptedException {

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if ((robotPose.position.x.value() < 18.0) && (robotPose.position.y.value() > -48))  {
                return 40.0;
            } else {
                return 80.0;
            }
        };

        Pose2d initialPose = new Pose2d(25, -62, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose); //initialize drive
        BridgeArmClaw bac = new BridgeArmClaw(hardwareMap);//initial all servos

        //Trajectories
        TrajectoryActionBuilder pushSamplesTraj = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(48, -10))
                .strafeToConstantHeading(new Vector2d(48,-46))
                .strafeToConstantHeading(new Vector2d(48, -10))
                .strafeToConstantHeading(new Vector2d(56,-13))
                .strafeToConstantHeading(new Vector2d(56,-46))
                .strafeToConstantHeading(new Vector2d(56, -13))
                .strafeToConstantHeading(new Vector2d(61, -13))
                .strafeToConstantHeading(new Vector2d(61, -46)) //new TranslationalVelConstraint(200))
                .strafeToConstantHeading(new Vector2d(38, -59));
        //.splineToConstantHeading(new Vector2d(17, -56), Math.toRadians(240));
        //step 1
        TrajectoryActionBuilder specimen1Traj = pushSamplesTraj.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-3, -31), Math.toRadians(0), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(-3, -31), baseVelConstraint); // -40
        //step 2
        TrajectoryActionBuilder specimen2TrajBack = specimen1Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(33, -52), new TranslationalVelConstraint(15));
        TrajectoryActionBuilder specimen2TrajHang = specimen2TrajBack.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-2.5, -31), Math.toRadians(0), baseVelConstraint); // -38

        //step 3
        TrajectoryActionBuilder specimen3TrajBack = specimen2TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(33, -53), new TranslationalVelConstraint(15));

        TrajectoryActionBuilder specimen3TrajHang = specimen3TrajBack.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-2, -31), Math.toRadians(0), baseVelConstraint); // -35

        //step 4
        TrajectoryActionBuilder specimen4TrajBack = specimen3TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        TrajectoryActionBuilder specimen4TrajHang = specimen4TrajBack.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-1.5, -31), Math.toRadians(0), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(-1.5, -31), baseVelConstraint); // -33

        //step 5
        TrajectoryActionBuilder specimen5TrajBack = specimen4TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);
        //    .strafeToConstantHeading(new Vector2d(33,-54), new TranslationalVelConstraint(15)); //34

        TrajectoryActionBuilder specimen5TrajHang = specimen5TrajBack.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-1, -31), Math.toRadians(0), baseVelConstraint);
        //.strafeToConstantHeading(new Vector2d(-1, -31), baseVelConstraint); // -31

        //park
        TrajectoryActionBuilder parkTraj = specimen5TrajHang.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        waitForStart();//wait until start is pressed

        if (isStopRequested()) return; //exit when stop is pressed

        //Sequential Actions + Parallel

        /* push samples (3 samples)
            1. reset rail and swing arm back
            2. do the driving path */
        SequentialAction pushSamples = new SequentialAction(
                new ParallelAction(
                        pushSamplesTraj.build()
                )
        );

        /* Clip Specimen #1 (pre-wall specimen) (repeat 1-5)
            1. close claw (claw should already be opened)
            2. go to hang specimen then come back */
      /*  SequentialAction w = new SequentialAction(
                bac.resetRailArm(),
                bac.closeSpecimenClaw()
        );*/
        SequentialAction w = new SequentialAction(
                bac.rrTest()
        );


        SequentialAction hangSpecimen1 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction(
                        specimen1Traj.build(),
                        bac.hangSpecimen()
                )

        );

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

        // Park
        SequentialAction park = new SequentialAction(
                new ParallelAction(
                        bac.resetRailArm(),
                        parkTraj.build()
                )

        );

        //run actions
        Actions.runBlocking(
                new SequentialAction(
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