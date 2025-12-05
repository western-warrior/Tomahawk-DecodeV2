//package org.firstinspires.ftc.teamcode.tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.fsm.EmergencyFSM;
//import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//@TeleOp
//public class TeleOp extends LinearOpMode {
//
//    private GamepadMappings controls;
//    private EmergencyFSM fsm;
//    private Robot robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize controls first
//        controls = new GamepadMappings(gamepad1, gamepad2);
//
//        // Initialize Robot before FSM!
//        robot = new Robot(hardwareMap, controls);
//
//        // Initialize FSM with fully constructed Robot
//        fsm = new EmergencyFSM(telemetry, controls, robot);
//
//        telemetry.addLine("Initialization complete");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//        }
//    }
//}
