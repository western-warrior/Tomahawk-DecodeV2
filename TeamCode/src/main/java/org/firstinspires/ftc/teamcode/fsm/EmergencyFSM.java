package org.firstinspires.ftc.teamcode.fsm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Config
public class EmergencyFSM {

    private Intake intake;
    private Outtake outtake;
//    private RTPAxon turret;           // turret controlled by RTPAxon
    private Turret turret;
    private Robot robot;
    private GamepadMappings controls;
    private Telemetry telemetry;
    private GazelleState gazelleState;
    private Intake transfer;

    public static int velocity = 1600;
    public EmergencyFSM(Telemetry telemetry, GamepadMappings controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.turret = robot.turret;
        this.outtake = robot.outtake;
        this.controls = controls;
        this.telemetry = telemetry;
        this.transfer = robot.transfer;
        this.gazelleState = GazelleState.BASE_STATE;
    }

    // ---------------- Main update loop ----------------
    public void gazelleUpdate() {
        controls.update();
        robot.driveTrain.update();
        robot.drive.localizer.update();

        // ---------------- Outtake / Flywheel ----------------
        if (controls.flywheelClose.value()) {
            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY);
        } else if (controls.flywheelFar.value()) {
            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY);
        } else if (controls.autoVelo.value()) {
//            outtake.shootVelocity(outtake.autoVelocity(robot.drive.localizer.getPose()));
            outtake.shootVelocity(velocity);
        } else {
            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
        }

        // ---------------- Intake / Transfer ----------------
        if (controls.intake.locked()) {
            intake.intake();
            transfer.setPower(0);
        } else if (controls.intakeReverse.locked()) {
            intake.intakeReverse();
            transfer.setPower(-1);
        } else if (controls.transfer.locked()) {
            transfer.setPower(1);
            intake.intake();
        } else {
            intake.intakeStop();
            transfer.setPower(0);
        }

        // ---------------- Turret control ----------------
//        if (turret != null) {
//            boolean autoAim = true; // always auto-aim at the current target color
//
//            if (autoAim) {
////                turret.autoAlign(robot.drive.localizer.getPose());
//            } else {
//                // Manual control
//                double power = 0;
//                if (controls.turretLeft.locked()) turret.changeTargetAngle(-20);
//                else if (controls.turretRight.locked()) turret.changeTargetAngle(20);
//
//
//            }
//
//            turret.update(); // always update PID / rotation
//        }

        // ---------------- FSM State Updates ----------------
        switch (gazelleState) {
            case BASE_STATE: break;
            case INTAKING: break;
            case TRANSFERRING: break;
        }

        telemetry.update();
    }

    // ---------------- Getters / Setters ----------------
    public GazelleState getState() { return gazelleState; }
    public void setState(GazelleState newState) { gazelleState = newState; }

    public enum GazelleState {
        BASE_STATE, INTAKING, TRANSFERRING
    }
}
