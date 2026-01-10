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
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class Blue15Auto extends LinearOpMode implements FCV2 {

    public static double INTAKE_WAIT_TIME = 3.5;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1050;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_CLOSE_START);

        Action preload = drive.actionBuilder(FCV2.BLUE_CLOSE_START)
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
                .strafeTo(FCV2.PGP_BLUE_ARTIFACT)
                .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.PGP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))

                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .waitSeconds(0.85)

                .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
                .strafeTo(FCV2.PPG_BLUE_ARTIFACT)
                .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y, FCV2.BLUE_ARTIFACT_ANGLE))

                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .build();

        Action gate_score = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE)
                .build();

        Action gate_return = drive.actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE))
                .setTangent(Math.toRadians(285))
                .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(45))

                .build();


        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
                .strafeTo(FCV2.GPP_BLUE_ARTIFACT)

                .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y, FCV2.BLUE_ARTIFACT_ANGLE))

                //                .setReversed(true)
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)

                .build();

        Action park = drive.actionBuilder(new Pose2d(GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, BLUE_CLOSE_ANGLE))
                .strafeTo(new Vector2d(PGP_BLUE_ARTIFACT.x, PGP_BLUE_ARTIFACT.y-5))
                .build();


        waitForStart();
        if (isStopRequested()) return;

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

                        //FIRST SPIKE
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

                        new ParallelAction(
                                gate_score,
                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                        ),
                        new ParallelAction(
                                gate_return,
                                robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL)
                        ),

                        new SequentialAction(
                                robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),
                                robot.intake.intakeTransferTimeAction(SHOOTER_TIME)
                        ),

                        //SECOND SPIKE
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

                        //THIRD SPIKE

                        new ParallelAction(
                                artifact3,
                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
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
        PoseStorage.side = PoseStorage.SIDE.BLUE;
    }

}