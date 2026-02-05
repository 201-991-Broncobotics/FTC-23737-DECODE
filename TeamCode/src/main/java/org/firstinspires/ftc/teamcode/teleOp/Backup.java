package org.firstinspires.ftc.teamcode.teleOp;
import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Required import
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Settings;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Backup")
public class Backup extends LinearOpMode {
    TestColorSensor colorSensor = new TestColorSensor();
    Servo indexer;
    Servo kicky;
    DcMotor turnTable, motorFlyWheel, intake;
    DcMotorEx turretPower;
    private ElapsedTime clock = new ElapsedTime();
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    static final double tPR = 537.6;
    static final double tRPM = 450;
    double ticksPerSecond = (tRPM / 60.0) * tPR;
    private Limelight3A limelights;
    private IMU imu;
    private static final double tTP = 0.015;// this will change
    private static final double mP = 0.4;
    private static final double sT = 1.0; // degrees
    private boolean sL = true;
    private ElapsedTime searchTimer = new ElapsedTime();
    private static final double sP = 0.2;   // slow scan speed
    private static final double wT = 0.75; // seconds before switching direction
    boolean lastflywheelvar = false;
    int flywheelvar = 0;
    int intakevar = 0;
    boolean lastintakevar = false;
    int kickyvar = 0;
    boolean lastkickyvar = false;
    boolean shift = false;


    ElapsedTime waitTimer;
    int processStep = 4;


    //MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++
    public void runOpMode() {
        turretPower = hardwareMap.get(DcMotorEx.class, "turret_Motor");
        turretPower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretPower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fLM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fRM");
        backRightMotor = hardwareMap.get(DcMotor.class, "bRM");
        indexer = hardwareMap.get(Servo.class, "sServo");
        kicky = hardwareMap.get(Servo.class, "fly_Wheel");
        turretPower = hardwareMap.get(DcMotorEx.class, "turret_Motor");
        motorFlyWheel = hardwareMap.get(DcMotor.class, "mFly");
        turnTable = hardwareMap.get(DcMotor.class, "turn_Table");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        limelights = hardwareMap.get(Limelight3A.class, "limelights");
        limelights.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        // 6000 RPM
        // setPower(1) = 6000 rpm
        turretPower.setDirection(DcMotorSimple.Direction.REVERSE);
        turretPower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        colorSensor.init(hardwareMap, telemetry);
        colorSensor.getColors();


        waitForStart();
        limelights.start();


        while (opModeIsActive() && !isStopRequested()) {
            shift = gamepad1.left_bumper;
            //indexer.setPower(0.1);
            colorSensor.getColors();
            double max, forward, strafe, rotate, throttle, magnitude, angle, frontLeftPower, frontRightPower, backLeftPower, backRightPower;
            // Might slow down code a bit by declaring a new variable every time, so I moved it so it declares each variable once ^ at the start
            forward = (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            strafe = (gamepad1.left_stick_x);
            rotate = (gamepad1.right_stick_x);
            // One way of making driving smoother
            magnitude = Math.hypot(forward, strafe);
            angle = Math.atan2(forward, strafe); //change to forward, strafe if doesn't work
            magnitude = Math.pow(magnitude, 2) * Math.signum(magnitude);
            forward = magnitude * Math.sin(angle);
            strafe = magnitude * Math.cos(angle);

            frontLeftPower = forward + strafe + rotate;
            frontRightPower = forward - strafe - rotate;
            backLeftPower = forward - strafe + rotate;
            backRightPower = forward + strafe - rotate;

            max = Math.max(1, Math.max(Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.abs(backLeftPower)), Math.abs(backRightPower)));
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;


            LLResult llResult = limelights.getLatestResult();
            if (llResult != null && llResult.isValid()) {

                double tx = llResult.getTx();
                double turnPower = tx * tTP + .2;

                if (Math.abs(tx) < sT) {
                    turnPower = 0;
                }

                turnPower = Math.max(-mP, Math.min(mP, turnPower));

                turnTable.setPower(turnPower);

                telemetry.addData("tag detected", true);
                telemetry.addData("tx (deg)", tx);
                telemetry.addData("turn power", turnPower);

            } else {
                // no tag -> stop turntable
                if (searchTimer.seconds() > wT) {
                    sL = !sL;   // flip direction
                    searchTimer.reset();
                }

                telemetry.addData("Target RPM", tRPM);
                telemetry.addData("Actual RPM",
                        turretPower.getVelocity() * 60 / tPR);
                telemetry.update();


                if (gamepad1.left_stick_y == 0 && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }


                frontLeftMotor.setPower(frontLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backLeftMotor.setPower(backLeftPower);
                backRightMotor.setPower(backRightPower);
//            frontLeftMotor.setPower(frontLeftPower * throttle);
//            frontRightMotor.setPower(frontRightPower * throttle);
//            backLeftMotor.setPower(backLeftPower * throttle);
//            backRightMotor.setPower(backRightPower * throttle);


//TurnTable*******TurnTable*******TurnTable*******TurnTable*******TurnTable*******TurnTable*******
                float x = gamepad1.right_trigger;
                float y = gamepad1.left_trigger;


                if (x > 0) {
                    turnTable.setPower(x);
                }
                turnTable.setPower(0);

                if (y > 0) {
                    turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
                    turnTable.setPower(-y);
                }
                turnTable.setPower(0);


//FLYWHEEL*******FLYWHEEL*******FLYWHEEL*******FLYWHEEL*******FLYWHEEL*******FLYWHEEL*******``
                if (gamepad1.dpad_left && !lastflywheelvar) {
                    // Toggle the state
                    if (flywheelvar == 0) {
                        flywheelvar = 1;
                    } else {
                        flywheelvar = 0;
                    }
                }

                lastflywheelvar = gamepad1.dpad_left;

                if (flywheelvar == 1) {
                    turretPower.setVelocityPIDFCoefficients(Settings.FlywheelKP, Settings.FlywheelKI, Settings.FlywheelKD, 0);
                    //turretPower.setVelocity(ticksPerSecond);4
                    turretPower.setVelocity(Settings.flywheelVel / 60 * 20);
                } else {
                    turretPower.setPower(0);
                }
//INTAKE*******INTAKE*******INTAKE*******INTAKE*******INTAKE*******INTAKE*******INTAKE*******

                if (gamepad1.dpad_down && !lastintakevar) {
                    // Toggle the state
                    if (intakevar == 0) {
                        intakevar = 1;
                    } else {
                        intakevar = 0;
                    }
                }
                lastintakevar = gamepad1.dpad_down;

                if (intakevar == 1) {
                    if (shift) {
                        intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        intake.setPower(.8);
                    } else {
                        intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        intake.setPower(.8);
                    }

                } else {
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0);
                }


//KICKY*******KICKY*******KICKY*******KICKY*******KICKY*******KICKY*******KICKY*******KICKY*******
                if (gamepad1.dpad_up && !lastkickyvar) {
                    // Toggle the state
                    if (kickyvar == 0) {
                        kickyvar = 1;
                    } else {
                        kickyvar = 0;
                    }
                }

                lastkickyvar = gamepad1.dpad_up;
                if (kickyvar == 1) {
                    kicky.setPosition(.37);
                    motorFlyWheel.setPower(1);
                } else {
                    kicky.setPosition(.725);
                    motorFlyWheel.setPower(0);
                }


//SHIFT********SHIFT********SHIFT********SHIFT********SHIFT********SHIFT********SHIFT********SHIFT********
                if (gamepad1.left_bumper) {
                    shift = true;
                } else {
                    shift = false;
                }


//RAPID********RAPID********RAPID********RAPID********RAPID********RAPID********RAPID********RAPID********
                if (gamepad1.right_bumper) {
                    waitTimer.reset();
                    indexer.setPosition(.85);
                    kickyvar = 1;
                    processStep = 0;
                }
                if (waitTimer.time() > .8 && processStep == 0) {
                    indexer.setPosition(.5);
                    processStep += 1;
                }
                if (waitTimer.time() > 1.6 && processStep == 1) {
                    indexer.setPosition(.15);
                    processStep += 1;
                }
                if (waitTimer.time() > 2.4 && processStep == 2) {
                    indexer.setPosition(.67);
                    processStep += 1;
                    kickyvar = 0;
                }


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                // intake — 1 shooting — .5
                // intake — .30 shooting .85
                // intake — .67 shooting — .17/.15


                if (gamepad1.x) {
                    if (shift) {
                        indexer.setPosition(.85);
                    } else {
                        indexer.setPosition(.30);
                    }

                }
                if (gamepad1.y) {
                    if (shift) {
                        indexer.setPosition(.15);
                    } else {
                        indexer.setPosition(.67);
                    }
                }
                if (gamepad1.b) {
                    if (shift) {
                        indexer.setPosition(.5);
                    } else {
                        indexer.setPosition(1);
                    }
                }
           /*telemetry.addData("Servo position", indexer.getPosition());
           telemetry.addData("Target RPM", tRPM);
           telemetry.addData("Actual RPM", turretPower.getVelocity() * 60 / tPR);
           telemetry.update();


            */
            }
        }
    }
}