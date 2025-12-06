//package org.firstinspires.ftc.teamcode.tuners;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
//import org.firstinspires.ftc.teamcode.gamepad.Toggle;
//import org.firstinspires.ftc.teamcode.pid.MiniPID;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
//
//@TeleOp
//@Config
//public class ToggleTester extends LinearOpMode {
//
//
//
//
//    public void runOpMode() {
//        GamepadMappings controls = new GamepadMappings(gamepad1, gamepad2);
//        Toggle[] allControls = {controls.flywheelClose, controls.transfer, controls.intake, controls.intakeReverse, controls.}
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
//            telemetry.update();
//        }
//    }
//
//
//}