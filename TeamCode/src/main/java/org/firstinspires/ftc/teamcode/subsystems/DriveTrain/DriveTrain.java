package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IMUGyro;

public class DriveTrain {

    private final IMUGyro imu;

    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx bl;
    private final DcMotorEx br;

    private final DcMotorEx[] motors;
    private final DcMotorEx[] reversedMotors;

    private final LinearOpMode mode;

    public State s = State.ROBOTCENTRIC;




    public DriveTrain(LinearOpMode mode) {
        this.mode = mode;

        fl = mode.hardwareMap.get(DcMotorEx.class, "fl");
        fr = mode.hardwareMap.get(DcMotorEx.class, "fr");
        bl = mode.hardwareMap.get(DcMotorEx.class, "bl");
        br = mode.hardwareMap.get(DcMotorEx.class, "br");

        imu = new IMUGyro(mode);


        motors = new DcMotorEx[]{fl, fr, bl, br};
        reversedMotors = new DcMotorEx[]{bl, fl};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotorEx motor : reversedMotors) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }

    }

    public void update() {

        double y = -mode.gamepad1.left_stick_y;
        double x = mode.gamepad1.left_stick_x * 1.1;
        double rx = mode.gamepad1.right_stick_x;
        // double denominator;
        double flPower = 0.0;
        double frPower = 0.0;
        double blPower = 0.0;
        double brPower = 0.0;
        switch (s) {
            case ROBOTCENTRIC:
                flPower = (y + x + rx) ;
                frPower = (y - x - rx) ;
                blPower = (y - x + rx) ;
                brPower = (y + x - rx) ;
                break;

            case FIELDCENTRIC:
                double botHeading = imu.getYaw(AngleUnit.RADIANS);
                double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                flPower = (rotY + rotX + rx) ;
                frPower = (rotY - rotX - rx) ;
                blPower = (rotY - rotX + rx) ;
                brPower = (rotY + rotX - rx) ;
                break;
        }


        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void addTelemetry() {
        mode.telemetry.addData("Drivetrain State", s);
    }

    public void resetIMU() {
        imu.resetIMU();
    }

    public enum State {
        ROBOTCENTRIC, FIELDCENTRIC
    }
}