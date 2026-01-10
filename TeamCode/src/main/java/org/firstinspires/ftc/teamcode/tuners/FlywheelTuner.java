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
public class FlywheelTuner extends LinearOpMode {
    public static double VELOCITY = 1680;
    public static double POS = 0.86;
    public static double POWER = 0.77;
    public static boolean autoVelo = false;
    public static boolean intaking = true;
    public static boolean transfering = true;
    public static boolean setPower = false;
    public static boolean flywheelActive = true;


    public static double P = 430, I = 0, D = 1, F = 14.8;


    Outtake flywheel;
    Intake intake;
    MecanumDrive drive;


    public void runOpMode() {
        flywheel = new Outtake(this);
        intake = new Intake(this);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        flywheel.hood.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double currentVelocity = Math.abs(flywheel.getVelocity());
            double error = VELOCITY - currentVelocity;
            drive.localizer.update();


            if (intaking) {
                intake.intake();
            }
            else {
                intake.intakeStop();
            }
            if (transfering) {
                intake.transferIn(1);
            }
            else {
                intake.transferStop();
            }


            flywheel.motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            flywheel.motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));


            if (autoVelo) {
                flywheel.autoVelocity(drive.localizer.getPose());
            }
//            else {
//                flywheel.hood.setPosition(POS);
//            }
//            if (setPower) {
//                flywheel.motor1.setPower(POWER);
//                flywheel.motor2.setPower(POWER);
//            }
//            else {
//                flywheel.setVelocity(VELOCITY);
//            }






            telemetry.addData("Transfer State", intake.transfer.getDirection());
            telemetry.addData("Intake State", intake.intakeMotor.getDirection());
            telemetry.addData("Error", error);
            telemetry.addData("Velocity", currentVelocity);
            telemetry.addData("Set point", VELOCITY);
            telemetry.addData("BotX", drive.localizer.getPose().position.x);
            telemetry.addData("BotY", drive.localizer.getPose().position.y);
            telemetry.addData("AutoVelo", flywheel.autoVelo);
            telemetry.addData("AutoHoodPos", flywheel.autoHoodPos);
            telemetry.addData("currentHoodPos", flywheel.currentHoodPos);
            telemetry.addData("Distance", flywheel.distCalc(drive.localizer.getPose()));
            telemetry.addData("goalX", PoseStorage.goalX);
            telemetry.addData("goalY", PoseStorage.goalY);
            telemetry.update();
        }
    }




}

