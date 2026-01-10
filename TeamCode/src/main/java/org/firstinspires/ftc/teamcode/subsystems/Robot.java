package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public Robot(LinearOpMode mode) {
        driveTrain = new DriveTrain(mode);
        intake = new Intake(mode);
        turret = new Turret(mode);
        outtake = new Outtake(mode);
        transfer = new Intake(mode);
        drive = new MecanumDrive(mode.hardwareMap, new Pose2d(0, 0, 0));
    }

}