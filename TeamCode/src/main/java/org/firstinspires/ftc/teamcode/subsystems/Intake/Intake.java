package org.firstinspires.ftc.teamcode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    public DcMotorEx intakeMotor;
    public Servo blocker;

    public Intake(LinearOpMode mode) {

        intakeMotor = mode.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake() {
        intakeMotor.setPower(1);
    }
    public void intakeReverse() {
        intakeMotor.setPower(-0.5);
    }

    public void intakeStop() {
        intakeMotor.setPower(0);
    }

    public Action intakeTime(double time) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
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
                }

                else if (timer.seconds() >= time) {
                    intakeStop();
                    timer.reset();
                    return false;
                }


                return false;
            }
        };
    }

}
