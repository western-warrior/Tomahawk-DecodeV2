package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Close;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class Blue18Auto extends LinearOpMode implements FCV2 {

    public static double INTAKE_WAIT_TIME = 1.4;

    public static int ARTIFACT_SHOOT_VEL = 1500;


    public void runOpMode() throws InterruptedException {


        Robot robot = new Robot(this, null);
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

            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(45))
            .waitSeconds(0.85)

            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FCV2.PPG_BLUE_ARTIFACT)
            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y, FCV2.BLUE_ARTIFACT_ANGLE))

            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
            .build();

        Action gate_score = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE))
//                .strafeToLinearHeading(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE)
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE_INTAKE.x, FCV2.BLUE_GATE_INTAKE.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)
            .build();

        Action gate_return = drive.actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE))
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)

            .build();

        Action gate_score2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE))
//                .strafeToLinearHeading(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE)
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE_INTAKE.x, FCV2.BLUE_GATE_INTAKE.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)
            .build();

        Action gate_return2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE))
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)

            .build();

        Action gate_score3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE))
//                .strafeToLinearHeading(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE)
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_GATE_INTAKE.x, FCV2.BLUE_GATE_INTAKE.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)
            .build();

        Action gate_return3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE, FCV2.BLUE_GATE_INTAKE_ANGLE))
            .setTangent(Math.toRadians(285))
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), FCV2.BLUE_GATE_INTAKE_ANGLE)

            .build();

        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y-27), FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FCV2.GPP_BLUE_ARTIFACT)

            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y, FCV2.BLUE_ARTIFACT_ANGLE))

            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)

            .build();

        Action park = drive.actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-5))
            .build();

        Action a1 = new SequentialAction(
            new ParallelAction(
                artifact1
            ),
            new ParallelAction(
                artifact1_return
            ),
            robot.intake.intakeTransferTimeAction(INTAKE_WAIT_TIME)
        );

        Action a2 = new SequentialAction(
            new ParallelAction(
                artifact2
            ),
            new ParallelAction(
                artifact2_return
            ),

                robot.intake.intakeTransferTimeAction(INTAKE_WAIT_TIME)

        );

        Action a3 = new SequentialAction(
            new ParallelAction(
                artifact3
            ),
            new ParallelAction(
                artifact3_return
            ),
            robot.intake.intakeTransferTimeAction(INTAKE_WAIT_TIME)
        );



        Action gate = new SequentialAction(
            gate_score,
            new SleepAction(1.3),
            gate_return,
//            robot.intake.intakeTransferTimeAction(INTAKE_WAIT_TIME),
            new SleepAction(INTAKE_WAIT_TIME),
            gate_score2,
            new SleepAction(1.3),
            gate_return2,
//            robot.intake.intakeTransferTimeAction(INTAKE_WAIT_TIME)
            new SleepAction(INTAKE_WAIT_TIME)
        );


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new ParallelAction(
                new SequentialAction(

                    // PRELOAD

                    preload,

//                    new ParallelAction(
//                        robot.intake.intakeTransferTimeAction(INTAKE_WAIT_TIME)
//                    ),

                    // ARTIFACT 1

                    a1,

                    // GATE SCORES

                    gate,

                    // ARTIFACT 2

                    a2,

                    // ARTIFACT 3

                    a3,

                    // PARK

                    park
                ),
                robot.outtake.shootVelocityTimeAction(ARTIFACT_SHOOT_VEL, 29.9),
                robot.intake.intakeTimeAction(29.9)
            )

        );

    }
}
