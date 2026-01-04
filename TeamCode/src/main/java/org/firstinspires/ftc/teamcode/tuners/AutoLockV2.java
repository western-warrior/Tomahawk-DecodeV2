//package org.firstinspires.ftc.teamcode.tuners;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
//
//@TeleOp
//public class AutoLockV2 extends LinearOpMode {
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Turret turret = new Turret(hardwareMap);
//        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));
//
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            drive.localizer.update();
//            turret.autoAlign(drive.localizer.getPose());
//            turret.update();
//
//            telemetry.addData("Angle", turret.getAngle());
//            telemetry.addData("Target Angle", turret.getTargetAngle());
//            telemetry.addData("Error", turret.getError());
//            telemetry.addData("Power", turret.getPower());
//            telemetry.update();
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp(name = "Turret AutoLock")
@Config
public class AutoLockV2 extends LinearOpMode {

    /** Tunables */
    public static double pwr = 0.25;        // max servo power
    public static double targetAngle = 270; // degrees
    public static double tolerance = 3;     // degrees

    /** Utility functions */
    private double normalize(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    /** Returns shortest signed angle difference [-180, 180] */
    private double signedAngleDiff(double target, double current) {
        double diff = normalize(target) - normalize(current);
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }

    /** True if motion from current → target would cross the 180 hard stop */
    private boolean crosses180(double current, double target) {
        return (current < 180 && target > 180) ||
            (current > 180 && target < 180);
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, 24, 0));

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.localizer.update();

            double angle = normalize(encoder.getVoltage() / 3.3 * 360);
            double error = signedAngleDiff(targetAngle, angle);
            boolean illegalMove = crosses180(angle, targetAngle);

            double power;

            if (illegalMove || Math.abs(error) <= tolerance || pwr == 0) {
                power = 0;
            } else {
                power = Math.signum(error) * pwr;
            }

            left.setPower(power);
            right.setPower(power);

            telemetry.addData("Angle", angle);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Illegal Move", illegalMove);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
