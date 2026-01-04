package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp
@Config
public class AutoLock extends LinearOpMode {
    public static double minPower = 0.15;
    public static double maxPower = 0.6;
    public static double p = 0.008;
    public static double i = 0.0;      // Integral gain
    public static double d = 0.0003;
    public static double f = 0.0;      // Feedforward gain
    public static double targetAngle = 0;
    public static double tolerance = 5;
    public static double maxIntegral = 0.3;  // Prevent integral windup
    public static int servo = 0;
    public static boolean tuning = false;

    public static double encoderAtForward = 22;

    // These will be set based on initial position
    private double minBound;
    private double maxBound;
    private double initialAngle;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        AnalogInput servoEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));

        ElapsedTime timer = new ElapsedTime();
        double lastError = 0;
        double lastTime = 0;
        double integralSum = 0;  // Accumulated integral

        waitForStart();
        if (isStopRequested()) return;

        // Read initial position and set bounds
        double rawEncoder = (servoEncoder.getVoltage() / 3.3 * 360);
        initialAngle = rawEncoder - encoderAtForward;
        while (initialAngle > 180) initialAngle -= 360;
        while (initialAngle < -180) initialAngle += 360;

        // Set bounds: 180° in each direction from initial position
        minBound = initialAngle - 180;
        maxBound = initialAngle + 180;

        telemetry.addData("Initial Angle", initialAngle);
        telemetry.addData("Min Bound", minBound);
        telemetry.addData("Max Bound", maxBound);
        telemetry.addData("Press START to confirm", "");
        telemetry.update();

        sleep(2000); // Show bounds for 2 seconds

        timer.reset();

        while (opModeIsActive()) {
            drive.localizer.update();

            double currentTime = timer.seconds();
            double dt = currentTime - lastTime;

            // Read raw encoder (0-360)
            rawEncoder = (servoEncoder.getVoltage() / 3.3 * 360);

            // Convert to turret coordinates (-180 to 180, where 0 = forward)
            double currentAngle = rawEncoder - encoderAtForward;

            // Normalize to -180 to 180 range
            while (currentAngle > 180) currentAngle -= 360;
            while (currentAngle < -180) currentAngle += 360;

            // Clamp target to valid range based on initial position
            double clampedTarget = Math.max(minBound, Math.min(maxBound, targetAngle));

            // Calculate error
            double error = clampedTarget - currentAngle;

            // Normalize error to shortest path
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Calculate integral (accumulated error over time)
            if (dt > 0 && Math.abs(error) > tolerance) {
                integralSum += error * dt;
                // Anti-windup: clamp integral to prevent it from growing too large
                integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
            } else if (Math.abs(error) <= tolerance) {
                // Reset integral when at target
                integralSum = 0;
            }

            // Calculate derivative
            double derivative = 0;
            if (dt > 0) {
                derivative = (error - lastError) / dt;
            }

            // Calculate power with PIDF control
            double power = 0;

            if (Math.abs(error) > tolerance) {
                // PIDF control
                double proportional = error * p;
                double integral = integralSum * i;
                double derivativeTerm = derivative * d;
                double feedforward = Math.signum(error) * f;  // F provides constant push in error direction

                power = proportional + integral + derivativeTerm + feedforward;

                // Clamp to min/max power while preserving sign
                if (power > 0) {
                    power = Math.max(minPower, Math.min(maxPower, power));
                } else {
                    power = Math.max(-maxPower, Math.min(-minPower, power));
                }

                // Hard stop at bounds
                if ((currentAngle <= minBound + 10 && power < 0) ||
                        (currentAngle >= maxBound - 10 && power > 0)) {
                    power = 0;
                }
            } else {
                // At target, reset integral
                integralSum = 0;
            }

            if (tuning) {
                if (servo == 1) {
                    left.setPower(power);
                }
                else {
                    right.setPower(power);
                }
            }
            else {
                left.setPower(power);
                right.setPower(power);
            }

            // Update for next loop
            lastError = error;
            lastTime = currentTime;

            telemetry.addData("Raw Encoder", rawEncoder);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", clampedTarget);
            telemetry.addData("Min Bound", minBound);
            telemetry.addData("Max Bound", maxBound);
            telemetry.addData("Error", error);
            telemetry.addData("P Term", error * p);
            telemetry.addData("I Term", integralSum * i);
            telemetry.addData("D Term", derivative * d);
            telemetry.addData("F Term", Math.signum(error) * f);
            telemetry.addData("Integral Sum", integralSum);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}