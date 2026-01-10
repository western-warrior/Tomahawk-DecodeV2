package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.PoseStorage;

public class Turret {

    private boolean enabled = true; // allow manual override

    double power = 0;
    double error = 0;
    double currentAngle;
    double targetAngle = 0;
    double initialAngle;
    public double calculatedAngle;

    CRServo left;
    CRServo right;
    AnalogInput encoder;

    // Constructor
    public Turret(LinearOpMode mode) {
        left = mode.hardwareMap.get(CRServo.class, "turretLeft");
        right = mode.hardwareMap.get(CRServo.class, "turretRight");
        encoder = mode.hardwareMap.get(AnalogInput.class, "turretEncoderRight");
        initialAngle = 0;
    }

    // --------------- Auto-Align --------------

    public double autoAlign(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;

        double deltaX = PoseStorage.goalX - robotX;
        double deltaY = PoseStorage.goalY - robotY;
        calculatedAngle = Math.toDegrees(Math.atan2(deltaY * 1.15, deltaX * 1.15)) - Math.toDegrees(pose.heading.toDouble());
        setTargetAngle(calculatedAngle);
        return calculatedAngle;
    }

    // ---------------- Control ----------------
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setTargetAngle(double a) {
        targetAngle = a;
    }

    public void changeTargetAngle(double a) {
        targetAngle += a;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtTarget() {
        return error < 5;
    }

    public void update() {
        currentAngle = (((encoder.getVoltage() / 3.3 * 360) % 360) - 180) - initialAngle;
        targetAngle = (targetAngle > 180) ? targetAngle - 360 : targetAngle;

        error = (targetAngle - currentAngle) % 360;
        power = 0.2 * Math.log(1+Math.abs(error)) / Math.log(10);

        if (Math.abs(error) < 3 || Math.abs(Math.abs(error) - 360) < 3) {
            power = 0;
        } else {
            if ((error < 0)) {
                power = -power;
            }
        }
        if (enabled) left.setPower(power); else left.setPower(0.05);
        if (enabled) right.setPower(power); else right.setPower(0.05);
    }
}
