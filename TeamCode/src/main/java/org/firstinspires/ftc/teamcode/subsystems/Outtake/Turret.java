package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.PoseStorage;

public class Turret {

    public final RTPAxon leftServo;
    public final RTPAxon rightServo;

    private boolean enabled = true; // allow manual override

    double leftStart = 0;
    double rightStart = 0;

    MultipleTelemetry telemetry;
    // Constructor
    public Turret(HardwareMap hwMap) {
        CRServo left = hwMap.get(CRServo.class, "turretLeft");
        CRServo right = hwMap.get(CRServo.class, "turretRight");
        AnalogInput encoder = hwMap.get(AnalogInput.class, "turretEncoder");

        leftServo = new RTPAxon(left, encoder);
        rightServo = new RTPAxon(right, encoder);
        leftServo.setDirection(RTPAxon.Direction.REVERSE);
        rightServo.setDirection(RTPAxon.Direction.FORWARD);

        // Optional PID tuning
        leftServo.setPidCoeffs(0.015, 0.0005, 0.0025);
        rightServo.setPidCoeffs(0.015, 0.0005, 0.0025);

        leftServo.setMaxPower(0.8);
        rightServo.setMaxPower(0.8);

        leftStart = leftServo.getCurrentAngle();
        rightStart = rightServo.getCurrentAngle();
    }

    // --------------- Auto-Align --------------

    public void autoAlign(Pose2d pose) {
        update();
        leftServo.setRtp(true);
        rightServo.setRtp(true);

        double robotX = pose.position.x;
        double robotY = pose.position.y;

        double deltaX = PoseStorage.goalX - robotX;
        double deltaY = PoseStorage.goalY - robotY;
        double targetAngle = Math.toDegrees(Math.atan2(deltaX, deltaY) - pose.heading.toDouble());
        if (targetAngle < -360) targetAngle += 360;
        if (targetAngle > 360) targetAngle -= 360;
        leftServo.setTargetRotation(targetAngle);
        rightServo.setTargetRotation(-targetAngle);

    }

    // ---------------- Control ----------------
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setTargetAngle(double degrees) {
        if (enabled) {
            leftServo.setTargetRotation(degrees);
            rightServo.setTargetRotation(degrees);
        }
    }

    public void changeTargetAngle(double deltaDegrees) {
        if (enabled) {
            leftServo.changeTargetRotation(deltaDegrees);
            rightServo.changeTargetRotation(deltaDegrees);
        }
    }

    public void manualPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(-power);
        leftServo.setRtp(false);
        rightServo.setRtp(false);
    }

    public void update() {
        leftServo.update();
        rightServo.update();
    }

    public double getAngle() {
        return leftServo.getTotalRotation();
    }

    public boolean isAtTarget() {
        return leftServo.isAtTarget() && rightServo.isAtTarget();
    }

    public void reset() {
        leftServo.forceResetTotalRotation();
        rightServo.forceResetTotalRotation();
        leftServo.setRtp(true);
        rightServo.setRtp(true);
    }

    public String telemetryString() {
        return "\nLeft Error: " + (leftServo.getCurrentAngle() - leftServo.getTargetRotation()) + " | Right Error: " + (rightServo.getCurrentAngle() - rightServo.getTargetRotation());
    }
}
