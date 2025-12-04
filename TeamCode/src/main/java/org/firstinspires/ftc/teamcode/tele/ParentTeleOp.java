//package org.firstinspires.ftc.teamcode.tele;
//import org.firstinspires.ftc.teamcode.fsm.GazelleFSM;
//import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//@TeleOp
//public class ParentTeleOp extends LinearOpMode {
//    public static HardwareMap globalHardwareMap;
//    public static Telemetry globalTelemetry;
//    public static Gamepad globalGamepad1;
//    public static Gamepad globalGamepad2;
//    private GamepadMappings controls;
//    private GazelleFSM fsm;
//    private Robot robot;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        globalHardwareMap = hardwareMap;
//        globalTelemetry = telemetry;
//        globalGamepad1 = gamepad1;
//        globalGamepad2 = gamepad2;
//        controls = new GamepadMappings(globalGamepad1, globalGamepad2);
//        fsm = new GazelleFSM(telemetry, controls, robot);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            controls.update();
//            fsm.gazelleUpdate();
//
//
//        }
//    }
//}