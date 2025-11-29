package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage.side;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;
import org.firstinspires.ftc.teamcode.pid.MiniPID;

public class Turret {
    DcMotorEx turretMotor;
    MiniPID velocityController;
    public double pidOutput;
    private double aimAngle = 0;
    public static double P = 0, I = 0, D = 0, F = 0;
    private double lineX;
    private double lineY;
    private Rotation2d heading;
    private int turnAngle;
    public static double turretOffsetX = -1.34;
    public static double turretOffsetY = 1.05;
    public static int redClose = -45, blueClose = 45, redFar = 0, blueFar = 0;
    public int pos;
    public double error;
    public double tx;
    public double angle;
    public double TICKS_PER_DEGREE = 4.57777;


    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        velocityController = new MiniPID(P, I, D, F);

    }
    public void updatePID(double tx) {
        this.tx = tx;
        double currentPos = turretMotor.getCurrentPosition();


        error = degToTicks(tx);

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
    public int degToTicks(double deg) {
        return (int) (TICKS_PER_DEGREE*deg);
    }

    public double getVelocity() {
        return turretMotor.getVelocity();
    }
    public double getAimAngle() {return aimAngle;}
    public void autoAim(Pose2d botPos) {
        aimAngle = angleCalc(botPos);
        updatePID(aimAngle);
    }

    public double angleCalc(Pose2d pose) {
        if (side.equals(PoseStorage.SIDE.BLUE)) {
            angle = Math.atan2(72 - pose.position.x + turretOffsetX, 72 - pose.position.y + turretOffsetY) * 360 / Math.PI;
        } else if (side.equals(PoseStorage.SIDE.RED)) {
            angle = Math.atan2(-72 - pose.position.x + turretOffsetX, -72 - pose.position.y + turretOffsetY) * 360 / Math.PI;
        }
        angle -= pose.heading.log() * 360 / Math.PI;
        return angle;
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

    public Action turretBlueFar() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updatePID(blueFar);
                return false;
            }
        };
    }

    public Action turretRedClose() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updatePID(redClose);
                return false;
            }
        };
    }

    public Action turretRedFar() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                updatePID(redFar);
                return false;
            }
        };
    }
}
