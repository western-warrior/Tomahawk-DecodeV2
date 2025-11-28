package org.firstinspires.ftc.teamcode.autonomous.autos.Red.Close;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;

@Autonomous
@Config
public class RedCloseAuto extends LinearOpMode implements FieldConstants {

    public static double INTAKE_WAIT_TIME = 3;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1765;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_CLOSE_START);

        Action preload = drive.actionBuilder(RED_CLOSE_START)
                .strafeToLinearHeading(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE+Math.toRadians(5))
                .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(RED_CLOSE_SHOOT.x, RED_CLOSE_SHOOT.y, RED_CLOSE_ANGLE+Math.toRadians(5)))
//                .strafeToLinearHeading(PPG_RED_ARTIFACT, RED_ARTIFACT_ANGLE+Math.toRadians(10))
                .strafeToLinearHeading(PPG_RED_ARTIFACT, RED_ARTIFACT_ANGLE)
                .setTangent(Math.PI/2)
                .lineToY(PPG_RED_ARTIFACT.y-ARTIFACT_DIST-22, new TranslationalVelConstraint(90))

                .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(PPG_RED_ARTIFACT.x, PPG_RED_ARTIFACT.y-ARTIFACT_DIST-12, RED_ARTIFACT_ANGLE+Math.toRadians(10)))

                .strafeToLinearHeading(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE+Math.toRadians(11))
//                .waitSeconds(0.85)

                .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(RED_CLOSE_SHOOT.x, RED_CLOSE_SHOOT.y, RED_CLOSE_ANGLE+Math.toRadians(12)))
                .strafeToLinearHeading(PGP_RED_ARTIFACT, RED_ARTIFACT_ANGLE)
                .setTangent(Math.PI/2)
                .lineToY(PGP_RED_ARTIFACT.y-ARTIFACT_DIST-25, new TranslationalVelConstraint(90))

                .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(PGP_RED_ARTIFACT.x, PGP_RED_ARTIFACT.y-ARTIFACT_DIST-21, RED_ARTIFACT_ANGLE))

                .strafeTo(PGP_RED_ARTIFACT)
                .strafeToLinearHeading(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE+Math.toRadians(13))
//                .setTangent(180)
//                .splineTo(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE+Math.toRadians(13))

                .build();

        Action park = drive.actionBuilder(new Pose2d(RED_CLOSE_SHOOT.x, RED_CLOSE_SHOOT.y, RED_CLOSE_ANGLE+Math.toRadians(15)))
                .strafeTo(PGP_RED_ARTIFACT)
                .build();


        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(

                        botActions.preload_parallel_red(preload),

                        new ParallelAction(
//                                subsystems.outtake.shoot_close_time(SHOOTER_TIME),
                                robot.intake.intakeTimeAction(SHOOTER_TIME)

                        ),
                        robot.outtake.stopAction(),

                        new ParallelAction(
                                artifact1,

                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME),
                                robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME)
                        ),

                        robot.intake.stop(),


                        new ParallelAction(
                                artifact1_return,
//                                subsystems.intake.intakeReverse(0.5),
                                robot.outtake.reverseTimeAction(1)
                        ),
                        robot.outtake.stopAction(),

                        new SequentialAction(
                                robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                                robot.intake.intakeTimeAction(SHOOTER_TIME)
                        ),

                        robot.outtake.stopAction(),

                        new ParallelAction(
                                artifact2,

                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME+1),
                                robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME+1)
                        ),

                        robot.intake.stop(),

                        new ParallelAction(
                                artifact2_return,
//                                subsystems.intake.intakeReverse(0.5),
                                robot.outtake.reverseTimeAction(1)
                        ),

                        new SequentialAction(
                                robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                                robot.intake.intakeTimeAction(SHOOTER_TIME)
                        ),

                        park

//                        new ParallelAction(
//                                artifact3,
//                                subsystems.intake.intake(4)
//                        ),
//
//                        subsystems.intake.stop(),
//                        new ParallelAction(
//                                artifact3_return,
//                                subsystems.intake.intakeReverse(0.5),
//                                subsystems.outtake.reverseTimeAction(1)
//                        ),
//                        new SequentialAction(
//                                subsystems.outtake.shoot_close(),
//                                subsystems.intake.intake(5)
//                        )


                )

        );
        PoseStorage.currentPose = robot.pinpoint.getPose();
    }

}