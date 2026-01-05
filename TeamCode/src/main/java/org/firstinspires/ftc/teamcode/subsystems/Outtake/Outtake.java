package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.PoseStorage.goalX;
import static org.firstinspires.ftc.teamcode.PoseStorage.goalY;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Outtake {
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    public Servo hood;
    public double SETPOINT;
    MultipleTelemetry telemetry;

    // PIDF coefficients for flywheel velocity control
    public static double P = 430, I = 0, D = 1, F = 14.8;

    // Hood control constants (tune these)
    public static double K = 0.05; // saturation rate for hood function
    public static double T = 120;  // distance offset for hood function
    public static double MIN_HOOD = 0.2;  // servo min (0-1)
    public static double MAX_HOOD = 0.8;  // servo max (0-1)
    public static double hoodOffset = 0.25;
    public double autoHoodPos;
    public double autoVelo;
    public double currentHoodPos;

    public Outtake(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        motor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        hood = hardwareMap.get(Servo.class, "hood");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotorEx.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }

    public void setVelocity(double velocity) {
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }
    public void shootVelocity(int velocity) {
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));


        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);


    }
    public void shootStop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public double getVelocity() {
        return (motor1.getVelocity() + motor2.getVelocity()) / 2.0;
    }

    /**
     * Automatic hood calculation based on distance to goal
     * Returns a servo position (0-1)
     */
    public double hoodCalc(double distance) {
        double rawHood = -3.76768E-8 * Math.pow(distance, 4)
                + 0.0000154024 * Math.pow(distance, 3)
                - 0.00226345 * Math.pow(distance, 2)
                + 0.142616 * distance
                - 2.70691;
        BigDecimal bd = new BigDecimal(Double.toString(rawHood));
        bd = bd.setScale(2, RoundingMode.HALF_UP);
        double roundHood =  bd.doubleValue();
        return Math.min(Math.max(0.2, roundHood), 0.75);
    }

    /**
     * Calculates flywheel velocity based on distance and hoodvelocity
     * This is a tunable exponential formula
     */
    public int veloCalc(double distance) {
        double velocity = 0.00000128898 * Math.pow(distance, 4)
                + 0.000774931 * Math.pow(distance, 3)
                - 0.302351 * Math.pow(distance, 2)
                + 34.96837 * distance
                + 248.70575;
        return (int) velocity;
    }

    /**
     * Automatic velocity + hood control
     */
    public void autoVelocity(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Hood control
        double hoodPos = (hoodCalc(distance))-hoodOffset;
        hood.setPosition(hoodPos);
        autoHoodPos = hoodPos;

        // Velocity control
        int velocity = veloCalc(distance);
        shootVelocity(velocity);
        autoVelo = velocity;

        currentHoodPos = hood.getPosition();

        telemetry.addData("Distance", distance);
    }

    //============== ACTIONS =============
    public Action autoVelocityAction(double robotX, double robotY, double goalX, double goalY) {
        return new Action() {
            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) init = true;

                // Compute distance
                double dx = goalX - robotX;
                double dy = goalY - robotY;
                double distance = Math.sqrt(dx*dx + dy*dy);

                // Hood
                double hoodPos = hoodCalc(distance);
                hood.setPosition(hoodPos);

                // Flywheel
                int velocity = veloCalc(distance);
                shootVelocity(velocity);

                telemetryPacket.put("Distance", distance);
                telemetryPacket.put("Hood", hood.getPosition());
                telemetryPacket.put("Velocity", velocity);

                // Return true if flywheel is at target (within 50 rpm)
                double error = Math.abs(getVelocity() - velocity);
                telemetryPacket.put("Error", error);
                return error < 50;
            }
        };
    }

    // Automatic velocity + hood action with time limit
    public Action autoVelocityTimeAction(double robotX, double robotY, double goalX, double goalY, double timeLimit) {
        return new Action() {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    init = true;
                    timer.reset();
                }

                if (timer.seconds() < timeLimit) {
                    // Compute distance
                    double dx = goalX - robotX;
                    double dy = goalY - robotY;
                    double distance = Math.sqrt(dx*dx + dy*dy);

                    // Hood
                    double hoodPos = hoodCalc(distance);
                    hood.setPosition(hoodPos);

                    // Flywheel
                    int velocity = veloCalc(distance);
                    shootVelocity(velocity);

                    telemetryPacket.put("Distance", distance);
                    telemetryPacket.put("Hood", hoodPos);
                    telemetryPacket.put("Velocity", velocity);
                } else {
                    shootStop();
                }

                return timer.seconds() >= timeLimit;
            }
        };
    }
    public Action shootVelocityAction(int velocity) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
                motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));


                motor1.setVelocity(velocity);
                motor2.setVelocity(velocity);

                double error = Math.abs(getVelocity() - velocity);
                telemetryPacket.put("Error", error);
                return error < 50;
            }
        };
    }

    public Action stopAction() {
        return  new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                motor1.setPower(0);
                motor2.setPower(0);

                return true;
            }
        };
    }
}
