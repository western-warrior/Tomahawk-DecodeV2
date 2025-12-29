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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        double rawHood = Math.max(MIN_HOOD, Math.min(MAX_HOOD, distance * 0.01)); // this is a linear regression but we can change this if needed
        return Math.max(MIN_HOOD, Math.min(MAX_HOOD, rawHood));
    }

    /**
     * Calculates flywheel velocity based on distance and hood
     * This is a tunable exponential formula
     */
    public int veloCalc(double distance, double hoodPos) {
        double velocity = 1680; // it lwk gonna stay the same cs we have hood
        return (int) velocity;
    }

    /**
     * Automatic velocity + hood control
     *
     * @return
     */
    public int autoVelocity(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Hood control
        double hoodPos = hoodCalc(distance);
        hood.setPosition(hoodPos);

        // Velocity control
        int velocity = veloCalc(distance, hoodPos);
//        shootVelocity(velocity);

        telemetry.addData("Distance", distance);
        telemetry.addData("Hood", hoodPos);
        telemetry.addData("Velocity", velocity);
        return velocity;
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
                int velocity = veloCalc(distance, hoodPos);
                shootVelocity(velocity);

                telemetryPacket.put("Distance", distance);
                telemetryPacket.put("Hood", hoodPos);
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
                    int velocity = veloCalc(distance, hoodPos);
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
