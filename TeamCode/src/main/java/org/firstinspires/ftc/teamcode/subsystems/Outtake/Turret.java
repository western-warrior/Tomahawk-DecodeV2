package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pid.MiniPID;

public class Turret {
    DcMotorEx turretMotor;
    MiniPID velocityController;
    public double pidOutput;
    private int aimAngle = 0;
    public static double P = 0,I = 0, D = 0, F = 0;
    private double lineX;
    private double lineY;
    private Rotation2d heading;
    private int turnAngle;
    public static double turretOffsetX = 0;
    public static double turretOffsetY = 0;
    public static int redClose = -45, blueClose = 45, redFar = 0, blueFar = 0;
    public int pos;
    public double error;
    public double tx;
    public double TICKS_PER_DEGREE = 4.57777;


    public Turret(LinearOpMode mode) {
        turretMotor = mode.hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        velocityController = new MiniPID(P, I, D, F);

    }
    public void updatePID(double tx) {
        this.tx = tx;
        double currentPos = turretMotor.getCurrentPosition();


        error = tx*TICKS_PER_DEGREE;

        velocityController.setSetpoint(currentPos - error);
        pidOutput = velocityController.getOutput(error);

        pos = (int) pidOutput;
        turretMotor.setTargetPosition((int) pidOutput);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);


        velocityController.setP(P);
        velocityController.setI(I);
        velocityController.setD(D);
        velocityController.setF(F);

    }
    public int degToTicks(int deg) {
        return (int) Math.round(deg * (537.7/360) * 3);
    }

    public double getVelocity() {
        return turretMotor.getVelocity();
    }
    public void autoAim(Pose2d botPos) {
        aimAngle = angleCalc(botPos);
        updatePID(aimAngle);
    }

    public int angleCalc(Pose2d botPos) {
        lineX = 72 - (botPos.position.x+turretOffsetX);
        lineY = 72 - (botPos.position.y+turretOffsetY);
        heading = botPos.heading;
        aimAngle = (int) Math.round(Math.atan(lineX/lineY));
        return degToTicks(aimAngle);
    }

    public Action turretBlueClose() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updatePID(blueClose);
                return false;
            }
        };
    }
}
