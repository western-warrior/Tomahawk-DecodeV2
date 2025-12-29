package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;

public class Robot {

    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Intake transfer;

    public Turret turret;
    public MecanumDrive drive;
    public Robot(HardwareMap hwMap, GamepadMappings controls) {
        driveTrain = new DriveTrain(hwMap);
        intake = new Intake(hwMap);
        turret = new Turret(hwMap);
        outtake = new Outtake(hwMap);
        transfer = new Intake(hwMap);
        drive = new MecanumDrive(hwMap, PoseStorage.endPose);
    }

}