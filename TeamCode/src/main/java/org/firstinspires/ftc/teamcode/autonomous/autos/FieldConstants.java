package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public interface FieldConstants {


    double intake = 0;
    double shoot = 2;

    //-------------------------------Angles------------------------------------------

    //RED
    double RED_CLOSE_ANGLE = Math.toRadians(-47);
    double RED_FAR_ANGLE = Math.toRadians(90);
    double RED_ARTIFACT_ANGLE = Math.toRadians(270);

    //BLUE
    double BLUE_CLOSE_ANGLE = Math.toRadians(43);
    double BLUE_FAR_ANGLE = Math.toRadians(0);
    double BLUE_ARTIFACT_ANGLE = Math.toRadians(90);

    //-------------------------------Velocities------------------------------------------
    int CLOSE_VELOCITY = 1150;
    int FAR_VELOCITY = 1500;

    //-------------------------------Coordinates------------------------------------------

    // ======Artifact Length======
    double ARTIFACT_DIST = 36+5;
    double HP_ARTIFACT_DIST = 62;
    // ======RED ARTIFACTS======
//    Vector2d GPP_RED_ARTIFACT = new Vector2d(-36, -25);
//    Vector2d PGP_RED_ARTIFACT = new Vector2d(-12, -25);
//    Vector2d PPG_RED_ARTIFACT = new Vector2d(12, -54);

    Vector2d GPP_RED_ARTIFACT = new Vector2d(-36, -26);
    Vector2d PGP_RED_ARTIFACT = new Vector2d(-14, -32);
    Vector2d PPG_RED_ARTIFACT = new Vector2d(10, -52);
    Vector2d HP_RED_ARTIFACT = new Vector2d(-60,-58);


    // ======RED Shooting Locations======
    Vector2d RED_CLOSE_SHOOT = new Vector2d(15, -13);
    Pose2d RED_FAR_SHOOT = new Pose2d(-61.25, -5, Math.toRadians(-90));

    // ======BLUE ARTIFACTS======
    Vector2d GPP_BLUE_ARTIFACT = new Vector2d(-36, 26);
    Vector2d PGP_BLUE_ARTIFACT = new Vector2d(-14, 26);
    Vector2d PPG_BLUE_ARTIFACT = new Vector2d(10, 52);
    Vector2d HP_BLUE_ARTIFACT = new Vector2d(-60,58);


    // ======Blue Shooting Locations======
    Vector2d BLUE_CLOSE_SHOOT = new Vector2d(8,6);
    Pose2d BLUE_FAR_SHOOT = new Pose2d(-61.25, 5, Math.toRadians(0));


    // ======Gates======
    Vector2d BLUE_GATE = new Vector2d(-2, 55);
    Vector2d RED_GATE = new Vector2d(-2, -55);

    // ======STARTING POSITIONS======
    Pose2d RED_FAR_START = new Pose2d(-61.25, -5, Math.toRadians(0));

    Pose2d RED_CLOSE_START = new Pose2d(57, -45, Math.toRadians(-143));

    Pose2d BLUE_FAR_START = new Pose2d(-61.25, 5, Math.toRadians(0));

    Pose2d BLUE_CLOSE_START = new Pose2d(57, 45, Math.toRadians(323));

    Pose2d BLUE_GOAL_START = new Pose2d(-47.5, 51.5,  Math.toRadians(323));
}