package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.hardware.impl.MotorEx;

public class FlyWheelLogic {

    private DcMotorEx turretPower;

    private DcMotor intake;

    private DcMotor motorFlyWheel;

    private Servo kicky;

    private  Servo indexer;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum TurretPowerState{

        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET

    }

    private TurretPowerState turretPowerState;

    /// ///////////////////////////////////////// ///

    private double turretPowerVelocity = 0;

    private double MIN_TURRET_RPM = 800;

    private double TARGET_TURRET_RPM = 1100;

    private double TURRET_MAX_SPINUP_TIME = 2;

    /// ///////////////////////////////////////// ///

    private double INDEXER_1ST_POS = .85;

    private double INDEXER_2ND_POS = .5;

    private double INDEXER_3RD_POS = .15;

    private double INDEXER_4TH_POS = .67;

    /// ///////////////////////////////////////// ///

    private  double KICKY_CLOSE_ANGLE = .725;

    private double KICKY_OPEN_ANGLE = .37;

    private double KICKY_OPEN_TIME = .4;

    private double KICKY_CLOSE_TIME = .4;

    /// ///////////////////////////////////////// ///

    private int shotRemaining = 0;

    private double flywheelVelocity = 0;

    private double MIN_FLYWHEEL_RPM = 800;

    private double TARGET_FLYWHEEL_RPM = 1100;

    private double FLYWHEEL_MAX_SPINUP_TIME = 2;

    /// ///////////////////////////////////////// ///

    private double intakeVelocity = 0;

    private double MIN_INTAKE_RPM = 800;

    private double TARGET_INTAKE_RPY = 1100;

    private double INTAKE_MAX_SPINUP_TIME = 2;

    /// ///////////////////////////////////////// ///

    public void init(HardwareMap hwMap){
        kicky = hwMap.get(Servo.class,"fly_Wheel");
        turretPower = hwMap.get(DcMotorEx.class,"turret_Motor");
        indexer = hwMap.get(Servo.class, "sServo");
        intake = hwMap.get(DcMotor.class,"intake");

        turretPowerState = TurretPowerState.IDLE;

        turretPower.setPower(0);
        kicky.setPosition(KICKY_CLOSE_ANGLE);

    }

    public void update(){
        switch (turretPowerState){

            case IDLE:
                if (shotRemaining>0){
                    turretPower.setPower(TARGET_TURRET_RPM);
                    kicky.setPosition(KICKY_OPEN_ANGLE);

                    stateTimer.reset();
                    turretPowerState = TurretPowerState.SPIN_UP;

                }
                break;
            case SPIN_UP:
                if (turretPowerVelocity > MIN_TURRET_RPM || stateTimer.seconds() > TURRET_MAX_SPINUP_TIME){
                    kicky.setPosition(KICKY_CLOSE_ANGLE);
                    stateTimer.reset();

                    turretPowerState = TurretPowerState.LAUNCH;

                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > KICKY_CLOSE_TIME){
                    shotRemaining--;
                    kicky.setPosition(KICKY_OPEN_ANGLE);
                    stateTimer.reset();

                    turretPowerState = TurretPowerState.RESET;

                }
                break;
            case RESET:
                if (stateTimer.seconds() >  KICKY_CLOSE_TIME){
                    if (shotRemaining > 0){
                        stateTimer.reset();

                        turretPowerState = TurretPowerState.SPIN_UP;

                    }
                    else {
                        turretPower.setPower(0);
                        turretPowerState = TurretPowerState.IDLE;

                    }

                }
                break;
        }
    }
    private void fireShots(int numberOfShots){
        if (turretPowerState == turretPowerState.IDLE){
            shotRemaining =  numberOfShots;

        }
    }

    public boolean isBusy(){
        return  turretPowerState != TurretPowerState.IDLE;

    }
}
