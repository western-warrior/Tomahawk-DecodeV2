package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class Blue12 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setConstraints(80, 80, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();

        Action preload = myBot.getDrive().actionBuilder(FieldConstants.BLUE_CLOSE_START)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)
            .build();

        Action artifact1 = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FieldConstants.PPG_BLUE_ARTIFACT.x, FieldConstants.BLUE_CLOSE_SHOOT.y), FieldConstants.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FieldConstants.PPG_BLUE_ARTIFACT)
            .lineToY(FieldConstants.PPG_BLUE_ARTIFACT.y - 16)
            .strafeToLinearHeading(FieldConstants.BLUE_GATE, 0)
            .build();

        Action artifact1_return = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.BLUE_GATE.x, FieldConstants.BLUE_GATE.y, 0))

            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)
            .waitSeconds(0.85)

            .build();


        Action artifact2 = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.PGP_BLUE_ARTIFACT, FieldConstants.BLUE_ARTIFACT_ANGLE)

            .setTangent(FieldConstants.BLUE_ARTIFACT_ANGLE)
            //
            .lineToY(FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FieldConstants.PGP_BLUE_ARTIFACT.x, FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, FieldConstants.BLUE_ARTIFACT_ANGLE), +Math.PI/2.2)

            .build();

        Action artifact2_return = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.PGP_BLUE_ARTIFACT.x, FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, FieldConstants.BLUE_ARTIFACT_ANGLE))

//            .strafeTo(FieldConstants.PGP_BLUE_ARTIFACT)
//            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE+Math.toRadians(5+2))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE), Math.PI/8)

//                            .splineToLinearHeading(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE))
//            .setReversed(true)
//            .strafeTo(FieldConstants.BLUE_CLOSE_SHOOT)

            .build();



        Action artifact3 = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.GPP_BLUE_ARTIFACT, FieldConstants.BLUE_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FieldConstants.GPP_BLUE_ARTIFACT, +0.75*Math.PI)
            .waitSeconds(.2)
            .lineToY(FieldConstants.GPP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST)

            .build();

        Action artifact3_return = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.GPP_BLUE_ARTIFACT.x, FieldConstants.GPP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, FieldConstants.BLUE_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)

            .build();

        Action park = myBot.getDrive().actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FieldConstants.PGP_BLUE_ARTIFACT.x, FieldConstants.PGP_BLUE_ARTIFACT.y+5))
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
                    artifact3_return
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                ),
                park

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