package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d endPose;
    public static SIDE side;

    public static double goalX;
    public static double goalY;

    public enum SIDE {
        RED, BLUE
    }
}
