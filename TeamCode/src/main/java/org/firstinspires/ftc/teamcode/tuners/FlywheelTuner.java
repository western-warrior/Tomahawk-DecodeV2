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

    public static double FAR_VELOCITY = 1680;
    public static double CLOSE_VELOCITY = 1680;
    public static double POS = 0.2;

    public static double P = 500, I = 0, D = 0, F = 14.3;

    Outtake flywheel;
    Intake intake;
    MecanumDrive drive;

    public void runOpMode() {
        flywheel = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
//        MiniPID controller = new MiniPID(P, I, D, V);
        drive = new MecanumDrive(hardwareMap, new Pose2d(24, 24, 0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentVelocity = Math.abs(flywheel.getVelocity());
            double error = CLOSE_VELOCITY - currentVelocity;
//            double pidOutput = controller.getOutput(currentVelocity, CLOSE_VELOCITY);

//            pidOutput = Math.max(-1, Math.min(1, pidOutput));
            drive.localizer.update();
    //            intake.intake();
    //            intake.transferIn(1);
            flywheel.hood.setPosition(POS);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity", currentVelocity);
            telemetry.addData("Set point", CLOSE_VELOCITY);
//            telemetry.addData("PID Output", pidOutput);

            flywheel.motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            flywheel.motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

            flywheel.autoVelocity(drive.localizer.getPose());
            flywheel.setVelocity(CLOSE_VELOCITY);
            telemetry.addData("Transfer State", intake.transfer.getDirection());
            telemetry.addData("Intake State", intake.intakeMotor.getDirection());
            telemetry.update();
        }
    }


}