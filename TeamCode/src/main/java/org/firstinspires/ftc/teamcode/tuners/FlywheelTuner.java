package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pid.MiniPID;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;

@TeleOp
@Config
public class FlywheelTuner extends LinearOpMode {
    public static double VELOCITY = 1680;
    public static double POS = 0.2;

    public static double P = 430, I = 0, D = 1, F = 14.8;

    Outtake flywheel;
    Intake intake;
    MecanumDrive drive;

    public void runOpMode() {
        flywheel = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(24, 24, 0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentVelocity = Math.abs(flywheel.getVelocity());
            double error = VELOCITY - currentVelocity;
            drive.localizer.update();
            intake.intake();
            intake.transferIn(1);
            flywheel.hood.setPosition(POS);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity", currentVelocity);
            telemetry.addData("Set point", VELOCITY);

            flywheel.motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            flywheel.motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

//            flywheel.autoVelocity(drive.localizer.getPose());

            flywheel.setVelocity(VELOCITY);

            telemetry.addData("Transfer State", intake.transfer.getDirection());
            telemetry.addData("Intake State", intake.intakeMotor.getDirection());
            telemetry.update();
        }
    }


}