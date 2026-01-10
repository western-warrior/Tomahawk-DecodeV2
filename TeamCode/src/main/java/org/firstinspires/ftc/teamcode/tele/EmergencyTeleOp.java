package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fsm.EmergencyFSM;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class EmergencyTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize controls first
        GamepadMappings controls = new GamepadMappings(gamepad1, gamepad2);

        // Initialize Robot before FSM!
        Robot robot = new Robot(this);

        // Initialize FSM with fully constructed Robot
        EmergencyFSM fsm = new EmergencyFSM(telemetry, controls, robot);

        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controls.update();
            fsm.gazelleUpdate();

            // Optional telemetry
            telemetry.addData("FSM State", fsm.getState());
            telemetry.addData("Transfer speed", robot.transfer.getPower());
            telemetry.update();
        }
    }
}
