package org.firstinspires.ftc.teamcode.fsm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class EmergencyFSM {

    private Intake intake;
    private Outtake outtake;
//    private Turret turret;
    private Robot robot;
    private GamepadMappings controls;
    private Telemetry telemetry;
    private PinpointLocalizer pinpoint;
    private GazelleState gazelleState;
    private Intake transfer;
    // Constructor: only pass initialized Robot
    public EmergencyFSM(Telemetry telemetry, GamepadMappings controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
//        this.turret = robot.turret;
        this.pinpoint = robot.pinpoint;
        this.controls = controls;
        this.telemetry = telemetry;
        this.transfer = robot.transfer;
        this.gazelleState = GazelleState.BASE_STATE;
    }

    public void gazelleUpdate() {
        controls.update();
        robot.driveTrain.update();
        if (controls.flywheelClose.value()) {
            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY);
        }
        if (controls.flywheelOff.value()) {
            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
        }
//        else if (controls.flywheelFar.value()) {
//            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY);
//        }

        switch (gazelleState) {
            case BASE_STATE:
                intake.intakeStop();
                intake.transferStop();
                outtake.shootStop();

                if (controls.intake.locked() || controls.intakeReverse.locked()) {
                    gazelleState = GazelleState.INTAKING;
                }
                if (controls.transfer.value()) gazelleState = GazelleState.TRANSFERRING;
                break;

            case INTAKING:

                if (controls.intake.locked()) {intake.intake(); intake.transferIn(0);}
                else if (controls.intakeReverse.locked()) {intake.intakeReverse(); intake.transferOut(1);}
                else if (!controls.intake.locked()) {intake.intakeStop(); intake.transferStop();}

                else intake.intakeStop();
                 //if (controls.flywheel.value()) gazelleState = GazelleState.SPINUP;
                if (controls.transfer.locked()) gazelleState = GazelleState.TRANSFERRING;
                break;
/*
            case SPINUP:
                outtake.shootVelocity(1770);
                if (controls.servoBlocker.value()) gazelleState = GazelleState.TRANSFERRING;
                else if (controls.intake.locked() || controls.intakeReverse.locked()) gazelleState = GazelleState.INTAKING;
                else if (!controls.flywheel.value()) gazelleState = GazelleState.BASE_STATE;

                break;
*/
            case TRANSFERRING:
                transfer.setPower(1);
                intake.intake();
                //if (controls.intake.locked()) intake.intake();
                //else if (controls.intakeReverse.locked()) gazelleState = GazelleState.INTAKING;
//                else intake.intakeStop();
                if (!controls.transfer.locked()) gazelleState = GazelleState.INTAKING; intake.transferStop();
                break;
        }
    }

    public GazelleState getState() { return gazelleState; }
    public void setState(GazelleState newState) { gazelleState = newState; }

    public enum GazelleState {
        BASE_STATE, INTAKING, TRANSFERRING
    }
}
