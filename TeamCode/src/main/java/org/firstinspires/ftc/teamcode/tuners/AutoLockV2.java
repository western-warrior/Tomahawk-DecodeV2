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
    public static double pwr = 0.2;
    public static double targetAngle = 270;

    public void runOpMode() throws InterruptedException {
        SET_AS_BLUE.set();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        AnalogInput servoEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));

        double initialAngle = 0;
        double currentAngle = 0;
        double error;
        double power;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.localizer.update();
            error = Math.abs((currentAngle - (initialAngle + targetAngle)) % 360);
            error = ((currentAngle < 180 && targetAngle > 180) || (currentAngle > 180 && targetAngle < 180)) ? 360 - error : error;
            power = 0.25 * Math.log1p(error);
            currentAngle = (servoEncoder.getVoltage() / 3.3 * 360) % 360;


            if (Math.abs(error) < 5 || Math.abs(Math.abs(error) - 360) < 5) {
                left.setPower(0);
                right.setPower(0);
            } else {
                if ((currentAngle < 180 && targetAngle > 180) || (targetAngle < currentAngle && currentAngle < 180) || (targetAngle < currentAngle && targetAngle > 180)) {
                    //go left
                    left.setPower(pwr);
                    right.setPower(pwr);
                } else {
                    //go right
                    left.setPower(-pwr);
                    right.setPower(-pwr);
                }
            }

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.addData("pwr", power);
            telemetry.addData("Calculated Angle", turret.calculatedAngle);
            telemetry.update();
        }
    }
}