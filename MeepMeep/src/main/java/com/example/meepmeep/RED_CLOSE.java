package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class RED_CLOSE {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setConstraints(70, 70, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();

        Action preload = myBot.getDrive().actionBuilder(FieldConstants.RED_CLOSE_START)
            .strafeToLinearHeading(FieldConstants.RED_CLOSE_SHOOT, FieldConstants.RED_CLOSE_ANGLE)
            .build();

        Action artifact1 = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.RED_CLOSE_SHOOT.x, FieldConstants.RED_CLOSE_SHOOT.y, FieldConstants.RED_CLOSE_ANGLE))
            .turn(Math.toRadians(-48))
            .strafeTo(FieldConstants.PPG_RED_ARTIFACT)
//            .strafeToLinearHeading(FieldConstants.RED_GATE, 0)
            .build();

        Action artifact1_return = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.PPG_RED_ARTIFACT.x, FieldConstants.PPG_RED_ARTIFACT.y, FieldConstants.RED_CLOSE_ANGLE-Math.toRadians(45)))

            .strafeToLinearHeading(FieldConstants.RED_CLOSE_SHOOT, FieldConstants.RED_CLOSE_ANGLE)
            .waitSeconds(0.85)

            .build();


        Action artifact2 = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.RED_CLOSE_SHOOT.x, FieldConstants.RED_CLOSE_SHOOT.y, FieldConstants.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.PGP_RED_ARTIFACT, FieldConstants.RED_ARTIFACT_ANGLE)

            .setTangent(FieldConstants.RED_ARTIFACT_ANGLE)
            //
            .lineToY(FieldConstants.PGP_RED_ARTIFACT.y-FieldConstants.ARTIFACT_DIST)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FieldConstants.PGP_RED_ARTIFACT.x, FieldConstants.PGP_RED_ARTIFACT.y-FieldConstants.ARTIFACT_DIST, FieldConstants.RED_ARTIFACT_ANGLE), -Math.PI/2.2)

            .build();

        Action artifact2_return = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.PGP_RED_ARTIFACT.x, FieldConstants.PGP_RED_ARTIFACT.y-FieldConstants.ARTIFACT_DIST, FieldConstants.RED_ARTIFACT_ANGLE))

//            .strafeTo(FieldConstants.PGP_RED_ARTIFACT)
//            .strafeToLinearHeading(FieldConstants.RED_CLOSE_SHOOT, FieldConstants.RED_CLOSE_ANGLE-Math.toRadians(5-2))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FieldConstants.RED_CLOSE_SHOOT.x, FieldConstants.RED_CLOSE_SHOOT.y, FieldConstants.RED_CLOSE_ANGLE), Math.PI/8)

//                            .splineToLinearHeading(new Pose2d(FieldConstants.RED_CLOSE_SHOOT, FieldConstants.RED_CLOSE_ANGLE))
//            .setReversed(true)
//            .strafeTo(FieldConstants.RED_CLOSE_SHOOT)

            .build();



        Action artifact3 = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.RED_CLOSE_SHOOT.x, FieldConstants.RED_CLOSE_SHOOT.y, FieldConstants.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.GPP_RED_ARTIFACT, FieldConstants.RED_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FieldConstants.GPP_RED_ARTIFACT, -0.75*Math.PI)
            .waitSeconds(.2)
            .lineToY(FieldConstants.GPP_RED_ARTIFACT.y-FieldConstants.ARTIFACT_DIST)

            .build();

        Action artifact3_return = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.GPP_RED_ARTIFACT.x, FieldConstants.GPP_RED_ARTIFACT.y-FieldConstants.ARTIFACT_DIST, FieldConstants.RED_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FieldConstants.RED_CLOSE_SHOOT, FieldConstants.RED_CLOSE_ANGLE)

            .build();

        Action park = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.GPP_RED_ARTIFACT.x, FieldConstants.GPP_RED_ARTIFACT.y-FieldConstants.ARTIFACT_DIST, FieldConstants.RED_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FieldConstants.PGP_RED_ARTIFACT.x + 5, FieldConstants.PGP_RED_ARTIFACT.y))
            .build();

        myBot.runAction(
            new SequentialAction(
                preload,
                artifact1,
//                botActions.preload_parallel_blue(preload),
//
//                botActions.shoot_parallel(),
//
//                robot.outtake.stopAction(),
//
//
//
//
//
//
//
//                botActions.intake_parallel(artifact1),
//
//                robot.intake.stop(),

                new ParallelAction(
                    artifact1_return
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//
////                        robot.outtake.reverseAction(.5),
////                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                ),
//                robot.outtake.stopAction(),


//                new SequentialAction(
//                    //                                subsystems.outtake.shoot_close(),
//                    new ParallelAction(
////                        robot.outtake.shootVelocityTimeAction(CLOSE_VELOCITY, SHOOTER_TIME),
////                        robot.intake.intakeTimeAction(SHOOTER_TIME)
//                    )
//
//                ),

//                robot.outtake.stopAction(),





                // ARTIFACT 2

                new ParallelAction(
                    artifact2
//                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME),
//                    robot.outtake.shootVelocityTimeAction(CLOSE_VELOCITY, INTAKE_WAIT_TIME)
                ),
//                robot.intake.stop(),

                new ParallelAction(
                    artifact2_return
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//
////                        robot.outtake.reverseTimeAction(.5),
////                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                ),

//                new SequentialAction(
//                    //                                subsystems.outtake.shoot_close(),
//                    new ParallelAction(
////                        robot.outtake.shootVelocityTimeAction(CLOSE_VELOCITY, SHOOTER_TIME),
////                        robot.intake.intakeTimeAction(SHOOTER_TIME)
//                    )
//
//                ),
//                robot.outtake.stopAction(),





                // ARTIFACT 3

                new ParallelAction(
                    artifact3
//                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME+.5),
//                    robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME+.5)
                ),
//                robot.intake.stop(),

                new ParallelAction(
                    park
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                )

//                new SequentialAction(
//                    robot.intake.intakeTimeAction(SHOOTER_TIME)
//                )
            )

        );

        BufferedImage bg = null;
        try {
            bg = ImageIO.read(new File("MeepMeep/src/main/java/com/example/meepmeep/DECODE.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        meepMeep.setBackground(bg)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();


    }
}