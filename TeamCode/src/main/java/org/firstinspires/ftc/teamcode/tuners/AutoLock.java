//package org.firstinspires.ftc.teamcode.tuners;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp
//@Config
//public class AutoLock extends LinearOpMode {
//
//    public static boolean lock = true;
//
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        FtcDashboard dash = FtcDashboard.getInstance();
//        final MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));
//        List<Action> runningActions = new ArrayList<>();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            runningActions = new ArrayList<>();
//            drive.localizer.update();
//            double angle = Math.toRadians(45);
//            telemetry.addData("angle", drive.localizer.getPose().heading.toDouble() - angle);
//            telemetry.addData("pose x", drive.localizer.getPose().position.x);
//            telemetry.addData("pose y", drive.localizer.getPose().position.y);
//            telemetry.addData("pose heading", drive.localizer.getPose().heading.toDouble());
//            TelemetryPacket packet = new TelemetryPacket();
//            // updated based on gamepads
//            if (lock && Math.abs(angle - drive.localizer.getPose().heading.toDouble()) > 0.1) {
//                runningActions.add(
//                        drive.actionBuilder(drive.localizer.getPose())
//                            .turn(angle - drive.localizer.getPose().heading.toDouble())
//                            .build()
//
//                );
//            } else {
//                runningActions = new ArrayList<>();
//            }
//            // update running actions
//            List<Action> newActions = new ArrayList<>();
//            telemetry.addData("new actions", newActions);
//            telemetry.addData("running actions", runningActions);
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//
//            dash.sendTelemetryPacket(packet);
//            telemetry.update();
//        }
//    }
//
//
//}

package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class AutoLock extends LinearOpMode {

    public static boolean lock = true;


    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Intake intake = new Intake(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intake.intake();
            intake.transferIn(1);
            drive.localizer.update();
            double distance = Math.sqrt(Math.pow(36 - drive.localizer.getPose().position.x, 2) + Math.pow(36 - drive.localizer.getPose().position.y, 2));
            int velocity = outtake.autoVelocityTest(distance);
            outtake.shootVelocity(velocity);
            telemetry.addData("Dist", distance);
            telemetry.addData("Velocity", velocity);
            telemetry.update();
        }
    }


}