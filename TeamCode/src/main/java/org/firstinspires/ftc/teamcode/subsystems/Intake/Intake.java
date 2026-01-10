package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.BLOCKER_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.BLOCKER_OPEN;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public final DcMotorEx intakeMotor;
    public final DcMotorEx transfer;
    private final Telemetry telemetry;

    // Optional: pass telemetry if you want dashboard/logs
    public Intake(LinearOpMode mode, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motor
        intakeMotor = mode.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize motor
        transfer = mode.hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Overloaded constructor if telemetry is not needed
    public Intake(LinearOpMode mode) {
        this(mode, mode.telemetry);
    }

    // Motor controls
    public void intake() {
        intakeMotor.setPower(1);
    }

    public void intakeReverse() {
        intakeMotor.setPower(-1);
    }

    public void intakeStop() {
        intakeMotor.setPower(0);
    }

    // Blocker controls
    public void transferIn(double power) {
        if (transfer != null) transfer.setPower(power);
        if (telemetry != null) telemetry.addData("Transfer", "Forward");
    }

    public void transferOut(double power) {
        if (transfer != null) transfer.setPower(-power);
        if (telemetry != null) telemetry.addData("Transfer", "Reverse");
    }

    public void transferStop() {
        if (transfer != null) transfer.setPower(0);
        if (telemetry != null) telemetry.addData("Transfer", "Stopped");
    }

    // Actions for timing sequences
    public Action intakeTimeAction(double time) {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    intake();
                    timer.reset();
                    init = true;
                }
                if (timer.seconds() < time) {
                    return true;
                } else {
                    intakeStop();
                    transferStop();
                    return false;
                }
            }
        };
    }

    public Action intakeReverseTimeAction(double time) {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    intakeReverse();
                    timer.reset();
                    init = true;
                }
                if (timer.seconds() < time) {
                    return true;
                } else {
                    intakeStop();
                    transferStop();
                    return false;
                }
            }
        };
    }

    public Action intakeTransferTimeAction(double seconds) {
        return new Action() {
            ElapsedTime t = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                transferIn(1);
                intake();
                return (t.seconds() < seconds);
            }
        };
    }

    public Action stop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeStop();
                transferStop();
                return false;
            }
        };
    }

    public double getPower() {
        return transfer.getPower();
    }

    public void setPower(int i) {
        transfer.setPower(i);
    }
}
