package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;

public class Robot {

    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Turret turret;
    public PinpointLocalizer pinpoint;
    public Robot(LinearOpMode mode) {

        driveTrain = new DriveTrain(mode);
        intake = new Intake(mode);
        turret = new Turret(mode);
        outtake = new Outtake(mode);
        pinpoint = new PinpointLocalizer(mode.hardwareMap, 0.0025, PoseStorage.currentPose);

    }



}