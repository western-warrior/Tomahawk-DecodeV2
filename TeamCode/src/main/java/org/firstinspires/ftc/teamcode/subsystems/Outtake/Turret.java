package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.PoseStorage;

public class Turret {

    private boolean enabled = true; // allow manual override

    double power = 0;
    double error = 0;
    double angle = 0;
    double targetAngle = 0;
    double initialAngle;
    public double calculatedAngle;

    CRServo left;
    CRServo right;
    AnalogInput encoder;

    // Constructor
    public Turret(HardwareMap hwMap) {
        left = hwMap.get(CRServo.class, "turretLeft");
        right = hwMap.get(CRServo.class, "turretRight");
        encoder = hwMap.get(AnalogInput.class, "turretEncoder");
        initialAngle = encoder.getVoltage() / 3.3 * 360;
    }

    // --------------- Auto-Align --------------

    public void autoAlign(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;

        double deltaX = PoseStorage.goalX - robotX;
        double deltaY = PoseStorage.goalY - robotY;
        calculatedAngle = (Math.toDegrees(Math.atan2(deltaX, deltaY) - pose.heading.toDouble()));

        setTargetAngle(calculatedAngle);
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

    public double getAngle() {
        return angle;
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
        error = (angle > 180 && targetAngle > 180) ? (angle - (initialAngle + targetAngle)) % 360 : ((angle > 180 && targetAngle < 180) ? (360 - ((angle - (initialAngle + targetAngle))) % 360) : ((angle < 180 && targetAngle < 180) ? -((angle - (initialAngle + targetAngle)) % 360) : 360 + ((angle - (initialAngle + targetAngle)) % 360)));
        power = 0.25 * Math.log1p(error);
        angle = (encoder.getVoltage() / 3.3 * 360) % 360;

        boolean boundsHittingLeft = angle > 180 && angle < 360 && targetAngle < 180;
        boolean boundsHittingRight = angle < 180 && angle > 0  && targetAngle > 180;

        if (Math.abs(error) < 5 || Math.abs(Math.abs(error) - 360) < 5) {
            left.setPower(0);
            right.setPower(0);
        } else {
            if ((error > 180 || (error < 0 && error > -180) || (boundsHittingLeft)) && (!boundsHittingRight)) {
                left.setPower(power);
                right.setPower(power);
            } else {
                left.setPower(-power);
                right.setPower(-power);
            }
        }
    }
}
