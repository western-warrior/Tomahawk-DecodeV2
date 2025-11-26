package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pid.MiniPID;

public class Outtake {
    DcMotorEx flywheel1;
    DcMotorEx flywheel2;
    MiniPID velocityController;
    public double pidOutput;
    double error;
    public double SETPOINT;

    public static double P = 0,I = 0, D = 0, F = 0;


    public Outtake(LinearOpMode mode) {
        flywheel1 = mode.hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = mode.hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        velocityController = new MiniPID(P, I, D, F);
    }
    public double getVelocity() {
        return flywheel1.getVelocity();
    }
    public void setPower(double power) {
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    public void shootVelocity(int velocity, double Rx, double Ry, double hoodPos) {
        velocity = veloCalc (Rx, Ry, hoodPos);
        velocityController.setSetpoint(velocity);
        pidOutput = velocityController.getOutput(Math.abs(getVelocity()));
        setPower(pidOutput);
    }
    public void shootStop() {
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }
    public int veloCalc (double Rx, double Ry, double hoodPos){
        hoodPos = hoodCalc(Rx, Ry);
        int velocity = 0;
        return velocity;
    }
    public double hoodCalc(double Rx, double Ry){
        double hoodPos = 0;
        return hoodPos;
    }

    public Action shoot_velocity(int vel) {

        return new Action() {

            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!init) {
                    init = true;
                }


                SETPOINT = vel;
                velocityController.setSetpoint(SETPOINT);
                error = SETPOINT - Math.abs(getVelocity());

                pidOutput = velocityController.getOutput(Math.abs(getVelocity()));

                setPower(pidOutput);

//                setVelocity(1650);


                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));
                telemetryPacket.put("ERROR", error);
                telemetryPacket.put("SETPOINT", SETPOINT);


                return error >= 50;
            }

        };
    }


}
