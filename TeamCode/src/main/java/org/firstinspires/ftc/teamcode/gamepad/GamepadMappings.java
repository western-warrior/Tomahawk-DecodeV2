package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMappings {

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    //Write mappings here
    //Intake -
    //Intake Reverse -
    //Servo Blocker -
    //Transfer -
    //Transfer Reverse -
    //Flywheel -
    //Auto Aim Toggle -



    //=============== DRIVETRAIN ===============
    public static double drive = 0.0;
    public static double strafe = 0.0;
    public static double turn = 0.0;

    //=============== INTAKE ===============
    public Toggle intake;
    public Toggle intakeReverse;
    public Toggle servoBlocker;
    public Toggle transfer;
    public Toggle transferReverse;


    //=============== OUTTAKE ===============
    public Toggle flywheel;
    public Toggle autoAim;


    public GamepadMappings(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        //=============== INTAKE ===============
        intake = new Toggle(false);
        intakeReverse = new Toggle(false);
        servoBlocker = new Toggle(false);
        transfer = new Toggle(false);
        transferReverse = new Toggle(false);

        //=============== OUTTAKE ===============
        flywheel = new Toggle(false);
        autoAim = new Toggle(false);
    }

    public void joystickUpdate() {
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
    }

    public void intakeUpdate() {
        intake.update(gamepad1.right_trigger > 0.5);
        transfer.update(gamepad1.right_trigger > 0.5);
    }
    public void intakeReverseUpdate() {
        intakeReverse.update(gamepad1.left_trigger > 0.5);
        transferReverse.update(gamepad1.left_trigger > 0.5);
    }
    public void outtakeUpdate() {
        flywheel.update(gamepad2.left_stick_button);
        autoAim.update(gamepad1.dpad_up);
    }

    // v1 robot
    public void update() {
        joystickUpdate();

        intakeUpdate();

        servoBlocker.update(gamepad1.left_bumper);
    }

    public void resetControls(Toggle... toggles) {
        for (Toggle toggle : toggles) {
            toggle.set(false);
        }
    }
}

