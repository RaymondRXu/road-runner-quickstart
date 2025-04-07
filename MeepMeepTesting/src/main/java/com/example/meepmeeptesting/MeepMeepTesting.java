package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        Pose2d initialPose = new Pose2d(-3, -62, Math.toRadians(90));
        RoadRunnerBotEntity rightWithSpline = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if ((robotPose.position.x.value() < 18.0) && (robotPose.position.y.value() > -48))  {
                return 40.0;
            } else {
                return 80.0;
            }
        };

        TrajectoryActionBuilder pushSamples = rightWithSpline.getDrive().actionBuilder(initialPose)
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-3, -31), baseVelConstraint)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(24, -62, Math.toRadians(0)), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(24, -62), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(44, -10, Math.toRadians(90)), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(48, -10))
                .strafeToConstantHeading(new Vector2d(48,-46))
                //.strafeToConstantHeading(new Vector2d(48, -10))
                .splineToConstantHeading(new Vector2d(56,-13), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(56,-46))
                //.strafeToConstantHeading(new Vector2d(56, -13))
                .splineToConstantHeading(new Vector2d(61, -13), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(61, -46))
                .splineToConstantHeading(new Vector2d(38, -59), Math.toRadians(0));

        TrajectoryActionBuilder clipFirstSpec = pushSamples.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-3, -31), baseVelConstraint);

        TrajectoryActionBuilder pickUpSecondSpec = clipFirstSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        TrajectoryActionBuilder clipSecondSpec = pickUpSecondSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-2.5, -31), baseVelConstraint);

        TrajectoryActionBuilder pickUpThirdSpec = clipSecondSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        TrajectoryActionBuilder clipThirdSpec = pickUpThirdSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-1, -31), baseVelConstraint);

        TrajectoryActionBuilder pickUpFourthSpec = clipThirdSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        TrajectoryActionBuilder clipFourthSpec = pickUpFourthSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-0.5, -31), baseVelConstraint);

        TrajectoryActionBuilder pickUpFifthSpec = clipFourthSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(38, -59), baseVelConstraint);

        TrajectoryActionBuilder clipFifthSpec = pickUpFifthSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-0.5, -31), baseVelConstraint);

        TrajectoryActionBuilder park = clipFifthSpec.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(35, -59), baseVelConstraint);

        rightWithSpline.runAction(
                new SequentialAction(
                        pushSamples.build(),
                        clipFirstSpec.build(),
                        pickUpSecondSpec.build(),
                        clipSecondSpec.build(),
                        pickUpThirdSpec.build(),
                        clipThirdSpec.build(),
                        pickUpFourthSpec.build(),
                        clipFourthSpec.build(),
                        pickUpFifthSpec.build(),
                        clipFifthSpec.build(),
                        park.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightWithSpline)
                .start();
    }
}