package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
public class Robot {

    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Turret turret;
    public Robot(LinearOpMode mode) {
        
        //mode is erroring bc no code in subsystem classes
        driveTrain = new DriveTrain(mode);
        intake = new Intake(mode);
        turret = new Turret(mode);
        outtake = new Outtake(mode);

    }



}
