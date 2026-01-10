package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.tele.SET_AS_BLUE;

@TeleOp
@Config
public class AutoLock extends LinearOpMode {
    public static double targetAngle = 0;
    public static boolean enabled = false;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        Turret turret = new Turret(this);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoderRight");
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));

        double error;
        double power;
        double currentAngle;

        waitForStart();
        double initialAngle = (((encoder.getVoltage() / 3.3 * 360) % 360));


        while (opModeIsActive()) {
            drive.localizer.update();
            turret.autoAlign(drive.localizer.getPose());

//            currentAngle = (((encoder.getVoltage() / 3.3 * 360) % 360) - 180) - initialAngle;
//            targetAngle = turret.autoAlign(drive.localizer.getPose());
//            targetAngle = (targetAngle > 180) ? targetAngle - 360 : targetAngle;
//
//            error = (targetAngle - currentAngle) % 360;
//            power = 0.2 * Math.log(1+Math.abs(error)) / Math.log(10);
//
//            if (Math.abs(error) < 4 || Math.abs(Math.abs(error) - 360) < 4) {
//                power = 0;
//            } else {
//                if ((error < 0)) {
//                    //go clockwise
//                    power = -power;
//                }
//            }
//
//            if (enabled) left.setPower(power); else left.setPower(0);
//            if (enabled) right.setPower(power); else right.setPower(0);



            telemetry.addData("Current Angle", turret.getCurrentAngle());
            telemetry.addData("Initial Angle", 0);
            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Error", turret.getError());
            telemetry.addData("Power", turret.getPower());
            telemetry.addData("Rotation", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("X", drive.localizer.getPose().position.x);
            telemetry.addData("Y", drive.localizer.getPose().position.y);
            telemetry.update();
        }
    }
}