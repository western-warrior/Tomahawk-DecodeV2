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
import org.firstinspires.ftc.teamcode.pid.MiniPID;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.tele.SET_AS_BLUE;

@TeleOp
@Config
public class AutoLockV2 extends LinearOpMode {
    public static boolean enabled = false;
    public static double targetAngle = 270;

    public void runOpMode() throws InterruptedException {
        SET_AS_BLUE.set();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        AnalogInput servoEncoder = hardwareMap.get(AnalogInput.class, "turretEncoderLeft");
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));

        double initialAngle = 0;
        double currentAngle = 0;
        double realError;
        double error;
        double power;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.localizer.update();
            currentAngle = (servoEncoder.getVoltage() / 3.3 * 360) % 360;
            targetAngle = (targetAngle > 180) ? targetAngle - 360 : targetAngle;

            realError = (currentAngle - (initialAngle + targetAngle)) % 360;
            error = ((targetAngle <= 0 && currentAngle >= 0) || (targetAngle >= 0 && currentAngle <= 0)) ? 360 - Math.abs(realError) : realError;
            power = 0.25 * Math.log(1+Math.abs(error)) / Math.log(10);

            if (Math.abs(error) < 5 || Math.abs(Math.abs(error) - 360) < 5) {
                power = 0;
            } else {
                if ((currentAngle < 0 && targetAngle > 0) || (error < 180 && realError < 0)) {
                    //go right
                    power = -power;
                }
            }

            if (enabled) left.setPower(power); else left.setPower(0);
            if (enabled) right.setPower(power); else right.setPower(0);

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Real Error", realError);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Calculated Angle", turret.autoAlign(drive.localizer.getPose()));
            telemetry.update();
        }
    }
}