package org.firstinspires.ftc.teamcode.autonomous.autos.Red.Far;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;

@Autonomous
@Config
public class RedFarAuto extends LinearOpMode implements FIELD {

    public static double INTAKE_WAIT_TIME = 3;
    public static double SHOOTER_TIME = 2.5;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_FAR_START);


        Action artifact1 = drive.actionBuilder(new Pose2d(RED_FAR_START.position.x, RED_FAR_START.position.y, Math.toRadians(-90)))
//                .strafeToLinearHeading(PPG_RED_ARTIFACT, RED_ARTIFACT_ANGLE+Math.toRadians(10))
            .strafeToLinearHeading(GPP_RED_ARTIFACT, RED_ARTIFACT_ANGLE)
            .setTangent(Math.PI/2)
            .lineToY(GPP_RED_ARTIFACT.y-ARTIFACT_DIST+5, new TranslationalVelConstraint(90))

            .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(GPP_RED_ARTIFACT.x, GPP_RED_ARTIFACT.y-ARTIFACT_DIST+5, Math.toRadians(-90)))

            .strafeToLinearHeading(new Vector2d(RED_FAR_SHOOT.position.x, RED_FAR_SHOOT.position.y), Math.toRadians(-90))
//                .waitSeconds(0.85)

            .build();


        Action human = drive.actionBuilder(new Pose2d(RED_FAR_SHOOT.position.x, RED_FAR_SHOOT.position.y, Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(-24, HP_RED_ARTIFACT.y), Math.toRadians(180))
            .strafeTo(HP_RED_ARTIFACT)
            .build();

        Action human_return = drive.actionBuilder(new Pose2d(HP_RED_ARTIFACT.x, HP_RED_ARTIFACT.y, Math.toRadians(180)))
            .strafeToLinearHeading(new Vector2d(-48, HP_RED_ARTIFACT.y), Math.toRadians(180))
            .strafeToLinearHeading(new Vector2d(RED_FAR_SHOOT.position.x, RED_FAR_SHOOT.position.y), Math.toRadians(-90))
            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(RED_FAR_SHOOT.position.x, RED_FAR_SHOOT.position.y, Math.toRadians(-90)))
            .strafeToLinearHeading(PGP_RED_ARTIFACT, RED_ARTIFACT_ANGLE)
            .setTangent(Math.PI/2)
            .lineToY(PGP_RED_ARTIFACT.y-ARTIFACT_DIST+5, new TranslationalVelConstraint(90))

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(PGP_RED_ARTIFACT.x, PGP_RED_ARTIFACT.y-ARTIFACT_DIST+5, RED_ARTIFACT_ANGLE))

//            .strafeTo(new Vector2d(PGP_RED_ARTIFACT.x, PGP_RED_ARTIFACT.y-5))
            .setTangent(Math.toRadians(90))
            .splineTo(new Vector2d(RED_FAR_SHOOT.position.x, RED_FAR_SHOOT.position.y), Math.toRadians(-90))

            .build();




        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
            new SequentialAction(
                new SequentialAction(
                  robot.turret.turretRed(),
                  robot.outtake.shoot_far(),
                  robot.intake.intake(SHOOTER_TIME),
                  robot.outtake.shoot_stop(),
                    new ParallelAction(
                      artifact1,
                      robot.intake.intake(3)
                    ),
                    artifact1_return,
                    robot.outtake.shoot_far(),
                    robot.intake.intake(SHOOTER_TIME),
                    robot.outtake.shoot_stop(),
                    new ParallelAction(
                      human,
                      robot.intake.intake(5)
                    ),
                    human_return,
                    robot.outtake.shoot_far(),
                    robot.intake.intake(SHOOTER_TIME),
                    robot.outtake.shoot_stop()
                )
            )
        );
        PoseStorage.currentPose = robot.pinpoint.getPose();
    }

}
