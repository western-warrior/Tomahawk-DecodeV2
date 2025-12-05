package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Close;

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

import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;

@Autonomous
@Config
public class BlueCloseAuto extends LinearOpMode implements FieldConstants {

    public static double INTAKE_WAIT_TIME = 3;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1150;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, null);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_CLOSE_START);

        Action preload = drive.actionBuilder(FieldConstants.BLUE_CLOSE_START)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)
            .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FieldConstants.PPG_BLUE_ARTIFACT.x, FieldConstants.BLUE_CLOSE_SHOOT.y), FieldConstants.BLUE_ARTIFACT_ANGLE)
            .strafeTo(FieldConstants.PPG_BLUE_ARTIFACT)
            .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(FieldConstants.PPG_BLUE_ARTIFACT.x, FieldConstants.PPG_BLUE_ARTIFACT.y, FieldConstants.BLUE_ARTIFACT_ANGLE))

            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)
            .waitSeconds(0.85)
            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.PGP_BLUE_ARTIFACT, FieldConstants.BLUE_ARTIFACT_ANGLE)

            .setTangent(FieldConstants.BLUE_ARTIFACT_ANGLE)
            //
            .lineToY(FieldConstants.PGP_BLUE_ARTIFACT.y+FieldConstants.ARTIFACT_DIST+10)

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FieldConstants.PGP_BLUE_ARTIFACT.x, FieldConstants.PGP_BLUE_ARTIFACT.y-FieldConstants.ARTIFACT_DIST-10, FieldConstants.BLUE_ARTIFACT_ANGLE))

//             .setReversed(true)
            .lineToY(FieldConstants.PGP_BLUE_ARTIFACT.y)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)




            .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FieldConstants.BLUE_CLOSE_SHOOT.x, FieldConstants.BLUE_CLOSE_SHOOT.y, FieldConstants.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FieldConstants.GPP_BLUE_ARTIFACT, FieldConstants.BLUE_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FieldConstants.GPP_BLUE_ARTIFACT, -0.75*Math.PI)
            .waitSeconds(.2)
            .lineToY(FieldConstants.GPP_BLUE_ARTIFACT.y-26)

            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FieldConstants.GPP_BLUE_ARTIFACT.x, FieldConstants.GPP_BLUE_ARTIFACT.y-26, FieldConstants.BLUE_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .lineToY(FieldConstants.GPP_BLUE_ARTIFACT.y)
            .strafeToLinearHeading(FieldConstants.BLUE_CLOSE_SHOOT, FieldConstants.BLUE_CLOSE_ANGLE)

            .build();

        Action park = drive.actionBuilder(new Pose2d(BLUE_CLOSE_SHOOT.x, BLUE_CLOSE_SHOOT.y, BLUE_CLOSE_ANGLE-Math.toRadians(15)))
            .strafeTo(PGP_BLUE_ARTIFACT)
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

                         botActions.preload_parallel_red(preload),

//                         new ParallelAction(
//                                 robot.outtake.shootVelocityAction(1150),
//                                 robot.intake.intakeTimeAction(SHOOTER_TIME)
//
//                         ),
//                         robot.outtake.stopAction(),

                         new ParallelAction(
                                 artifact1
//                                 robot.outtake.shootVelocityAction(1150),
//                                 robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
//                                 robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME)

                         ),

                         robot.intake.stop(),


                         new ParallelAction(
                                 artifact1_return
//                                 robot.intake.intakeReverseTimeAction(0.5)
//                                 robot.outtake.reverseTimeAction(1)
                         ),
                         robot.outtake.stopAction(),

//                         new SequentialAction(
//                                 robot.outtake.shootVelocityAction(1150),
//                                 robot.intake.intakeTimeAction(SHOOTER_TIME)
//                         ),

                         robot.outtake.stopAction(),

                         new ParallelAction(
                                 artifact2

//                                 robot.intake.intakeTimeAction(INTAKE_WAIT_TIME-1)
//                                 robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME-1)
                         ),

                         robot.intake.stop(),

                         new ParallelAction(
                                 artifact2_return
//                                 robot.intake.intakeReverseTimeAction(0.5)
//                                 robot.outtake.reverseTimeAction(1)
                         ),

//                         new SequentialAction(
//                                 robot.outtake.shootVelocityAction(1150),
//                                 robot.intake.intakeTimeAction(SHOOTER_TIME)
//                         ),

//                         park

                         new ParallelAction(
                                 artifact3
//                                 robot.intake.intakeTimeAction(4)
                         ),

                         robot.intake.stop(),
                         new ParallelAction(
                                 artifact3_return
//                                 robot.intake.intakeReverseTimeAction(0.5)
//                                 robot.outtake.reverseTimeAction(1)
                         )
//                         new SequentialAction(
//                                 robot.outtake.shootVelocityAction(1150),
//                                 robot.intake.intakeTimeAction(5)
//                         )


                 )

         );
//         PoseStorage.currentPose = robot.pinpoint.getPose();
    }

}