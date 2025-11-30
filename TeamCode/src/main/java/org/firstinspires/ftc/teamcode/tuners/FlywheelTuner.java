package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pid.MiniPID;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;

@TeleOp
@Config
public class FlywheelTuner extends LinearOpMode {

    public static double FAR_VELOCITY = 2380;
    public static double CLOSE_VELOCITY = 1600;

    public static double P = 0.0025, I = 0, D = 0, V = 0.00035;

    Outtake flywheel;

    public void runOpMode() {
        flywheel = new Outtake(hardwareMap);
//        MiniPID controller = new MiniPID(P, I, D, V);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentVelocity = Math.abs(flywheel.getVelocity());
            double error = CLOSE_VELOCITY - currentVelocity;
//            double pidOutput = controller.getOutput(currentVelocity, CLOSE_VELOCITY);

//            pidOutput = Math.max(-1, Math.min(1, pidOutput));


            telemetry.addData("Error", error);
            telemetry.addData("Velocity", currentVelocity);
            telemetry.addData("Set point", CLOSE_VELOCITY);
//            telemetry.addData("PID Output", pidOutput);

            flywheel.motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new PIDFCoefficients(P, I, D, V));
            flywheel.motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new PIDFCoefficients(P, I, D, V));

            flywheel.setVelocity(CLOSE_VELOCITY);

            telemetry.update();
        }
    }


}