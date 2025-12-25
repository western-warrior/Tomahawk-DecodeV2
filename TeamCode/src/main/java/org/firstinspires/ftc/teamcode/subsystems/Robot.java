package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;

public class Robot {

    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Intake transfer;
//    public Turret turret;
    public PinpointLocalizer pinpoint;
    public MecanumDrive drive;
    public Robot(HardwareMap hwMap, GamepadMappings controls) {
        driveTrain = new DriveTrain(hwMap, controls);
        intake = new Intake(hwMap);
//        turret = new Turret(hwMap);
        outtake = new Outtake(hwMap);
        transfer = new Intake(hwMap);
        pinpoint = new PinpointLocalizer(hwMap, 0.0025, PoseStorage.currentPose);
        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));


    }

}