// /*
// package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Far;


// import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.roadrunner.Action;
// import com.acmerobotics.roadrunner.ParallelAction;
// import com.acmerobotics.roadrunner.Pose2d;
// import com.acmerobotics.roadrunner.SequentialAction;
// import com.acmerobotics.roadrunner.TranslationalVelConstraint;
// import com.acmerobotics.roadrunner.Vector2d;
// import com.acmerobotics.roadrunner.ftc.Actions;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
// import org.firstinspires.ftc.teamcode.subsystems.Robot;
// import org.firstinspires.ftc.teamcode.PoseStorage;

// @Autonomous
// @Config
// public class BlueFarAuto extends LinearOpMode implements FIELD {

//     public static double INTAKE_WAIT_TIME = 3;
//     public static double SHOOTER_TIME = 2.5;


//     public void runOpMode() throws InterruptedException {

//         Robot robot = new Robot(this);

//         MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_FAR_START);

//         Action artifact1 = drive.actionBuilder(new Pose2d(BLUE_FAR_START.position.x, BLUE_FAR_START.position.y, Math.toRadians(90)))
// //                .strafeToLinearHeading(PPG_BLUE_ARTIFACT, BLUE_ARTIFACT_ANGLE+Math.toRadians(10))
//             .strafeToLinearHeading(GPP_BLUE_ARTIFACT, BLUE_ARTIFACT_ANGLE)
//             .setTangent(Math.PI/2)
//             .lineToY(GPP_BLUE_ARTIFACT.y+ARTIFACT_DIST-5, new TranslationalVelConstraint(90))

//             .build();

//         Action artifact1_return = drive.actionBuilder(new Pose2d(GPP_BLUE_ARTIFACT.x, GPP_BLUE_ARTIFACT.y+ARTIFACT_DIST-5, Math.toRadians(90)))

//             .strafeToLinearHeading(new Vector2d(BLUE_FAR_SHOOT.position.x, BLUE_FAR_SHOOT.position.y), Math.toRadians(90))
// //                .waitSeconds(0.85)

//             .build();


//         Action human = drive.actionBuilder(new Pose2d(BLUE_FAR_SHOOT.position.x, BLUE_FAR_SHOOT.position.y, Math.toRadians(90)))
//             .strafeToLinearHeading(new Vector2d(-24, HP_BLUE_ARTIFACT.y), Math.toRadians(180))
//             .strafeTo(HP_BLUE_ARTIFACT)
//             .build();

//         Action human_return = drive.actionBuilder(new Pose2d(HP_BLUE_ARTIFACT.x, HP_BLUE_ARTIFACT.y, Math.toRadians(180)))
//             .strafeToLinearHeading(new Vector2d(-48, HP_BLUE_ARTIFACT.y), Math.toRadians(180))
//             .strafeToLinearHeading(new Vector2d(BLUE_FAR_SHOOT.position.x, BLUE_FAR_SHOOT.position.y), Math.toRadians(90))
//             .build();


//         Action artifact2 = drive.actionBuilder(new Pose2d(BLUE_FAR_SHOOT.position.x, BLUE_FAR_SHOOT.position.y, BLUE_FAR_ANGLE))
//             .strafeToLinearHeading(PGP_BLUE_ARTIFACT, BLUE_ARTIFACT_ANGLE)
//             .setTangent(Math.PI/2)
//             .lineToY(PGP_BLUE_ARTIFACT.y-ARTIFACT_DIST+5, new TranslationalVelConstraint(90))

//             .build();

//         Action artifact2_return = drive.actionBuilder(new Pose2d(PGP_BLUE_ARTIFACT.x, PGP_BLUE_ARTIFACT.y-ARTIFACT_DIST+8, BLUE_ARTIFACT_ANGLE))

//             .strafeTo(new Vector2d(PGP_BLUE_ARTIFACT.x, PGP_BLUE_ARTIFACT.y-5))
//             .setTangent(Math.toRadians(70))
//             .splineTo(new Vector2d(BLUE_FAR_SHOOT.position.x, BLUE_FAR_SHOOT.position.y), Math.toRadians(-110))

//             .build();




//         waitForStart();
//         if (isStopRequested()) return;


//         Actions.runBlocking(
//             new SequentialAction(
//                 new SequentialAction(
//                     robot.turret.turretBlue(),
//                     robot.outtake.shoot_far(),
//                     robot.intake.intake(SHOOTER_TIME),
//                     robot.outtake.shoot_stop(),
//                     new ParallelAction(
//                         artifact1,
//                         robot.intake.intake(3)
//                     ),
//                     artifact1_return,
//                     robot.outtake.shoot_far(),
//                     robot.intake.intake(SHOOTER_TIME),
//                     robot.outtake.shoot_stop(),
//                     new ParallelAction(
//                         human,
//                         robot.intake.intake(5)
//                     ),
//                     human_return,
//                     robot.outtake.shoot_far(),
//                     robot.intake.intake(SHOOTER_TIME),
//                     robot.outtake.shoot_stop()
//                 )
//             )
//         );
//         robot.pinpoint.update();
//         PoseStorage.endPose = robot.pinpoint.getPose();
//         PoseStorage.side = PoseStorage.SIDE.BLUE;
//     }

// }
// */