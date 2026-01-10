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
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pid.MiniPID;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;


@TeleOp
@Config
public class PositionTuner extends LinearOpMode {
    public static boolean autoVelo = false;
    public static boolean intaking = true;
    public static boolean transfering = true;
    public static boolean setPower = false;

    public static double P = 430, I = 0, D = 1, F = 14.8;


    Outtake flywheel;
    Intake intake;
    MecanumDrive drive;


    public void runOpMode() {
        intake = new Intake(this);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            drive.localizer.update();

            if (intaking) {
                intake.intake();
            } else {
                intake.intakeStop();
            }
            if (transfering) {
                intake.transferIn(1);
            } else {
                intake.transferStop();
            }

            telemetry.addData("X", drive.localizer.getPose().position.x);
            telemetry.addData("Y", drive.localizer.getPose().position.y);
            telemetry.addData("Heading", Math.toDegrees(drive.localizer.getPose().heading.log()));
            telemetry.update();
        }


    }
}

