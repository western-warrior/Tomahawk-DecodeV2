package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;

public class LimelightCamera {

    LinearOpMode mode;

    public Limelight3A limelight;

    LLResult result;
    double error;
    double pidOutput;
    Telemetry telemetry;

    Turret turret;

    public LimelightCamera(LinearOpMode mode) {
        this.mode = mode;
        turret = new Turret(mode);
        telemetry = new MultipleTelemetry(mode.telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = mode.hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(1);

        limelight.pipelineSwitch(1);
        limelight.start();

        FtcDashboard.getInstance().startCameraStream(limelight, 30);
    }

    public void addTelemetry() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            mode.telemetry.addData("tx", result.getTx());
            mode.telemetry.addData("ty", result.getTy());
            mode.telemetry.addData("Bot Pose", botPose.toString());
        }

    }

    public void autoAlign() {
        result = limelight.getLatestResult();

        double tx = result.getTx();
//        if (tx == 0) turret.setState(Turret.State.ZERO);
        turret.changeTargetAngle(tx);

        Pose3D botPose = result.getBotpose();
        telemetry.addData("tx", result.getTx());
    }
}