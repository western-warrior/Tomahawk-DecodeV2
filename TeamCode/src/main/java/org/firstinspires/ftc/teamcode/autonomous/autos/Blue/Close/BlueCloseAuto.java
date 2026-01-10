package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Close;

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
import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class BlueCloseAuto extends LinearOpMode implements FieldConstants {

    public static double INTAKE_WAIT_TIME = 4;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1050;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_CLOSE_START);

        Action preload = drive.actionBuilder(FieldConstants.BLUE_CLOSE_START)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)
            .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .turn(Math.toRadians(43))
            .strafeTo(new Vector2d(FieldConstants.PPG_BLUE_ARTIFACT.x, FieldConstants.BLUE_CLOSE_SHOOT.y))
            .strafeTo(FieldConstants.PPG_BLUE_ARTIFACT)
//            .strafeToLinearHeading(FieldConstants.BLUE_GATE, 0)
            .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(FieldConstants.PPG_BLUE_ARTIFACT.x, FieldConstants.PPG_BLUE_ARTIFACT.y, FieldConstants.BLUE_CLOSE_ANGLE+Math.toRadians(43)))

            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)

            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.PGP_BLUE_ARTIFACT, FieldConstants.BLUE_ARTIFACT_ANGLE)

            .setTangent(FieldConstants.BLUE_ARTIFACT_ANGLE)
            //
            .lineToY(FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FieldConstants.PGP_BLUE_ARTIFACT.x, FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, FieldConstants.BLUE_ARTIFACT_ANGLE), +Math.PI/2.2)

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FieldConstants.PGP_BLUE_ARTIFACT.x, FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, FieldConstants.BLUE_ARTIFACT_ANGLE))

//            .strafeTo(FieldConstants.PGP_BLUE_ARTIFACT)
//            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE+Math.toRadians(5+2))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE), Math.PI/8)

//                            .splineToLinearHeading(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE))
//            .setReversed(true)
//            .strafeTo(FieldConstants.BLUE_CLOSE_SHOOT)

            .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.GPP_BLUE_ARTIFACT, FieldConstants.BLUE_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FieldConstants.GPP_BLUE_ARTIFACT, +0.75*Math.PI)
            .waitSeconds(.2)
            .lineToY(FieldConstants.GPP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST)

            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FieldConstants.GPP_BLUE_ARTIFACT.x, FieldConstants.GPP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, FieldConstants.BLUE_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)

            .build();

        Action park = drive.actionBuilder(new Pose2d(GPP_BLUE_ARTIFACT.x, FieldConstants.GPP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST, BLUE_CLOSE_ANGLE))
            .strafeTo(new Vector2d(PGP_BLUE_ARTIFACT.x, PGP_BLUE_ARTIFACT.y-5))
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
                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                ),

                robot.intake.stop(),
//                new ParallelAction(
//                    artifact3_return,
//                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL)
//                ),
                park
//                new SequentialAction(
////                    robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
////                    robot.intake.intakeTransferTimeAction(2.5)
//                )


            )

        );
        robot.drive.localizer.update();
        PoseStorage.endPose = robot.drive.localizer.getPose();
        PoseStorage.side = PoseStorage.SIDE.BLUE;
    }

}