package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config // makes all of these variables show up on the ftc dashboard and lets you edit them while the robot is on without changing code
public class Settings {


    public static double flywheelVel = 80000; // max target Flywheel RPM
    //public static double flywheelVelBumper = 18500; // target Flywheel RPM for close
    public static boolean justTurnFlywheelOn = true;
    public static double FlywheelKP = 1.4, FlywheelKI = 0, FlywheelKD = 0.001;


    public static boolean isTuning = false;


    public static double testSpeed = 37500;
    //Ki Should remain 0, KP should be like 0.5-2 ish, KD sshould be like 0.0001 or something like that
    //public static double servoAngle = 0.39;


    public static double turret_P = 0.3;
    public static double turret_I = 0.0;
    public static double turret_D = 0.0002;


    //TODO: FINISH FOR REGRESSION
    //X: 0.5 Y: 63700
    //X: 0.872 Y: 70000
    //X: 0.6 Y: 65000
    //X: 0.7 Y: 67000
    //X: 1.0 Y: 72250
    //X: 1.1 Y: 75000
    //Equation: 7840.67269x^{2}+6003.85442x+58730.843
}

