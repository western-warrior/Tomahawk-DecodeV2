package org.firstinspires.ftc.teamcode.autonomous.autos.Red.Close;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class Red12Auto extends LinearOpMode implements FCV2 {

    //TODO: maybe make intake run for the entire thing

    public static double INTAKE_WAIT_TIME = 4.6;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1050;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, null);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_CLOSE_START);

        Action preload = drive.actionBuilder(FCV2.RED_CLOSE_START)
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV2.PPG_RED_ARTIFACT.x, FCV2.RED_CLOSE_SHOOT.y), FCV2.RED_ARTIFACT_ANGLE)
                .strafeTo(FCV2.PPG_RED_ARTIFACT)
                .lineToY(FCV2.PPG_RED_ARTIFACT.y + 16)
                .strafeToLinearHeading(FCV2.RED_GATE, 0)
                .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.RED_GATE.x, FCV2.RED_GATE.y, 0))
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)

            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.PGP_RED_ARTIFACT.x+2, FCV2.PGP_RED_ARTIFACT.y), FCV2.RED_ARTIFACT_ANGLE)

            .setTangent(FCV2.RED_ARTIFACT_ANGLE)
            //
            .lineToY(FCV2.PGP_RED_ARTIFACT.y-FCV2.ARTIFACT_DIST-1)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y-FCV2.ARTIFACT_DIST, FCV2.RED_ARTIFACT_ANGLE), -Math.PI/2.2)

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PGP_RED_ARTIFACT.x+2, FCV2.PGP_RED_ARTIFACT.y-FCV2.ARTIFACT_DIST-1, FCV2.RED_ARTIFACT_ANGLE))

//            .strafeTo(FCV2.PGP_RED_ARTIFACT)
//            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE-Math.toRadians(5-2))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE), RED_CLOSE_ANGLE)

//                            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE))
//            .setReversed(true)
//            .strafeTo(FCV2.RED_CLOSE_SHOOT)

            .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.GPP_RED_ARTIFACT, FCV2.RED_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FCV2.GPP_RED_ARTIFACT, -0.75*Math.PI)
            .waitSeconds(.2)
            .lineToY(FCV2.GPP_RED_ARTIFACT.y-FCV2.ARTIFACT_DIST-12)

            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_RED_ARTIFACT.x, FCV2.GPP_RED_ARTIFACT.y-FCV2.ARTIFACT_DIST-12, FCV2.RED_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)

            .build();

        Action park = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y-5))
            .build();


        waitForStart();
        if (isStopRequested()) return;
//        Actions.runBlocking(
//            drive.actionBuilder(new Pose2d(0, 0, 0))
//                .strafeTo(new Vector2d(0, 24))
//                .strafeTo(new Vector2d(0, 0))
//                .build());

        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                    preload,
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL)
                ),

                new ParallelAction(
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                    robot.intake.intakeTransferTimeAction(SHOOTER_TIME)

                ),
                robot.outtake.stopAction(),
                new ParallelAction(
                    artifact1,
                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                ),

                robot.intake.stop(),


                new ParallelAction(
                    artifact1_return,
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL)
                ),

                new SequentialAction(
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                    robot.intake.intakeTransferTimeAction(SHOOTER_TIME)
                ),

                robot.outtake.stopAction(),
                robot.outtake.stopAction(),
                robot.intake.stop(),
                new ParallelAction(
                    robot.intake.intakeTimeAction(SHOOTER_TIME),
                    artifact2
                ),

                robot.intake.stop(),

                new ParallelAction(
                    artifact2_return,
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL)
                ),

                new SequentialAction(
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                    robot.intake.intakeTransferTimeAction(SHOOTER_TIME)
                ),
                robot.outtake.stopAction(),
                robot.outtake.stopAction(),

                new ParallelAction(
                    artifact3,
                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME-0.8)
                ),

                robot.intake.stop(),
                new ParallelAction(
                    artifact3_return,
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL)
                ),
                new SequentialAction(
                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                    robot.intake.intakeTransferTimeAction(3)
                ),
                park

            )

        );
        robot.drive.localizer.update();
        PoseStorage.endPose = robot.drive.localizer.getPose();
        PoseStorage.side = PoseStorage.SIDE.RED;
    }

}