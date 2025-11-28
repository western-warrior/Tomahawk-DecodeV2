package org.firstinspires.ftc.teamcode.fsm;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;


public class GazelleFSM {

    private Intake intake;
    private Outtake outtake;
    private GamepadMappings controls;
    private Robot robot;
    private Turret turret;
    private Telemetry telemetry;
    private PinpointLocalizer pinpoint;
    private GazelleState gazelleState;
    public GazelleFSM(Telemetry telemetry, GamepadMappings controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.turret = robot.turret;
        this.pinpoint = robot.pinpoint;
        this.controls = controls;
        this.telemetry = telemetry;

        gazelleState = GazelleFSM.GazelleState.BASE_STATE;

    }
    public void gazelleUpdate(){
        controls.update();
        robot.driveTrain.update();

        robot.turret.autoAim(pinpoint.getPose());

        switch (gazelleState){
            case BASE_STATE:
                robot.intake.intakeStop();
                robot.intake.blockerClose();
                robot.outtake.shootStop();
                if (true /*right trigger*/){
                    gazelleState = GazelleState.INTAKING;
                } else if (true /*left trigger*/){
                    gazelleState = GazelleState.INTAKING;
                }

                //intake off, blocker closed, flywheel off
                break;
            case INTAKING:

                intake.blockerClose();
                if (true /*right trigger*/) {
                    intake.intake();
                } else if (true /*left trigger*/){
                    intake.intakeReverse();
                } else if (true /*left bumper*/){
                    gazelleState = GazelleState.TRANSFERRING;
                } else if (true /*click left joy stick*/){
                    gazelleState = GazelleState.SPINUP;
                } else {
                    intake.intakeStop();
                }
                //intake on, blocker closed
                break;
            case SPINUP:
                outtake.shootVelocity(pinpoint.getPose().position.x, pinpoint.getPose().position.y);
                if (true /*outtake done spinning, left bumper*/) {
                    gazelleState= GazelleState.TRANSFERRING;
                } else if (true /*right trigger*/){
                    gazelleState = GazelleState.INTAKING;
                } else if (true /*left trigger*/){
                    gazelleState = GazelleState.INTAKING;
                } else if(true /*boolean lft joystick*/){
                    gazelleState = GazelleState.BASE_STATE;
                }
                //flywheel on
                //if done and right trigger, allow transfer
                break;
            case TRANSFERRING:
                intake.blockerOpen();
                if (true /*hold right trigger*/){
                    intake.intake();
                } else if (true /*hold left trigger*/){
                    gazelleState=GazelleState.INTAKING;
                } else if (true /*left joystick click*/){
                    gazelleState = GazelleState.SPINUP;
                }
                else {
                    intake.intakeStop();
                }

                //intake on, blocker up, flywheel on
                break;
        }
    }
    public GazelleState getState(){
        return gazelleState;
    }
    public void setState (GazelleState newState) {
        gazelleState = newState;
    }

    public enum GazelleState {
        BASE_STATE("BASE_STATE"),
        INTAKING ("INTAKING"),
        SPINUP ("SPINUP"),
        TRANSFERRING ("TRANSFERRING");


        private String state;

        GazelleState (String stateName){
            state = stateName;
        }
        public String stateName() {
            return state;
        }
    }

}



