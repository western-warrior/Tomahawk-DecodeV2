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


public class Blue15 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, 2.5, 3, 18)
                .setDimensions(16.53, 18)
                .build();

        Action preload = drive.getDrive().actionBuilder(FCV2.BLUE_CLOSE_START)
            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
            .build();

        Action artifact1 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FCV2.PGP_BLUE_ARTIFACT)
            .build();

        Action artifact1_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.PGP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))

            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(45))
            .waitSeconds(0.85)

            .build();


        Action artifact2 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FCV2.PPG_BLUE_ARTIFACT)
            .build();

        Action artifact2_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y, FCV2.BLUE_ARTIFACT_ANGLE))

            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
            .build();

        Action gate_score = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE.x - 10, FCV2.BLUE_GATE.y, Math.toRadians(90)), Math.toRadians(90))
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE_INTAKE.x - 10, 60, 0), Math.toRadians(90))
            .strafeToConstantHeading(new Vector2d(FCV2.BLUE_GATE_INTAKE.x - 6, 60))
            .build();

        Action gate_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE.x - 6, 60, Math.toRadians(90)))
            .setTangent(Math.toRadians(270))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(45))

            .build();

        Action gate_score2 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE.x - 10, FCV2.BLUE_GATE.y, Math.toRadians(90)), Math.toRadians(90))
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE_INTAKE.x - 10, 60, 0), Math.toRadians(90))
            .strafeToConstantHeading(new Vector2d(FCV2.BLUE_GATE_INTAKE.x - 6, 60))
            .build();

        Action gate_return2 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE.x - 6, 60, Math.toRadians(90)))
            .setTangent(Math.toRadians(270))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(45))

            .build();


        Action gate_score3 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE))
//                .strafeToLinearHeading(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE)
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE_INTAKE.x, FCV2.BLUE_GATE_INTAKE.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)
            .build();

        Action gate_return3 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE))
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)

            .build();

        Action artifact3 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FCV2.GPP_BLUE_ARTIFACT)

            .build();

        Action artifact3_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y, FCV2.BLUE_ARTIFACT_ANGLE))

            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)

            .build();

        Action park = drive.getDrive().actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-5))
            .build();

        drive.runAction(
                new SequentialAction(
                        preload,
                        artifact1,

                        new ParallelAction(
                                artifact1_return
                        ),

                        gate_score,
                        gate_return,

                        gate_score,
                        gate_return,

                        gate_score,
                        gate_return,

                        // ARTIFACT 2

                        new ParallelAction(
                                artifact2
                        ),

                        new ParallelAction(
                                artifact2_return
                        ),
                        // ARTIFACT 3

                        new ParallelAction(
                                artifact3
                        ),

                        new ParallelAction(
                                artifact3_return
                        )
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
                .addEntity(drive)
                .start();


    }
}