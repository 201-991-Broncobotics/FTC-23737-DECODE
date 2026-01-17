package org.firstinspires.ftc.teamcode.teleOp;

public class EMERGANCYCODE {
}//brush

/*
package org.firstinspires.ftc.teamcode.teleOp;
import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Required import
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//brush

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.concurrent.TimeUnit;

import dev.nextftc.hardware.impl.ServoEx;


@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends LinearOpMode { //if opmode isn't working, change back to linearopmode. It can be found in the emergancy code
    TestColorSensor colorSensor = new TestColorSensor();
    CRServo movingServo;
    Servo flyWheel;
    DcMotor turnTable, turretPower, motorFlyWheel, motorIntake;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    ElapsedTime clock;
    double finalTime = 0;


    boolean clockStarted = false;


    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fLM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fRM");
        backRightMotor = hardwareMap.get(DcMotor.class, "bRM");
        movingServo = hardwareMap.get(CRServo.class, "sServo");
        flyWheel = hardwareMap.get(Servo.class, "fly_Wheel");
        turretPower = hardwareMap.get(DcMotor.class, "turret_Motor");
        motorFlyWheel = hardwareMap.get(DcMotor.class, "mFly");
        turnTable = hardwareMap.get(DcMotor.class, "turn_Table");
        //motorIntake = hardwareMap.get(DcMotor.class, "intake");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        motorFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        movingServo.setDirection(DcMotorSimple.Direction.FORWARD);
        turnTable.setDirection(DcMotorSimple.Direction.FORWARD);
       // motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor.init(hardwareMap, telemetry);
        colorSensor.getColors();
        float x = gamepad1.right_trigger;
        float y = gamepad1.left_trigger;

        clock = new ElapsedTime();

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            movingServo.setPower(1);
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


            if (x > 0) {
                turnTable.setPower(x);
            }
            turnTable.setPower(0);

            if (y > 0) {
                turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
                turnTable.setPower(-y);
            }
            turnTable.setPower(0);


            if (gamepad1.dpad_up) {
                flyWheel.setPosition(.37);
                motorFlyWheel.setPower(1);
            } else if (!gamepad1.dpad_up) {
                flyWheel.setPosition(.725);
                motorFlyWheel.setPower(0);
            }

            if (gamepad1.dpad_left) {
                turretPower.setDirection(DcMotorSimple.Direction.REVERSE);
                turretPower.setPower(1);
            }
            if (gamepad1.dpad_right) {
                turretPower.setPower(0);
            }


                 while (gamepad1.a) {
                     movingServo.setPower(0);
                 }
                 while (gamepad1.b) {
                      movingServo.setPower(-1);
                 }



            TestColorSensor.DetectedColor detected = colorSensor.getColors();
            if (detected == TestColorSensor.DetectedColor.GREEN) {
                     movingServo.setPower(0);


            } else if (detected == TestColorSensor.DetectedColor.PURPLE) {

                    clock.reset();
                    clock.startTime();
                    if (clock.time(TimeUnit.SECONDS) >= 5) {
                        turretPower.setPower(1);
                        flyWheel.setPosition(.735);
                        motorFlyWheel.setPower(1);
                    }
                    if (clock.time(TimeUnit.SECONDS) >= 10) {
                        turretPower.setPower(0);
                        flyWheel.setPosition(.37);
                        motorFlyWheel.setPower(0);
                        if (clock.time(TimeUnit.SECONDS) >= 11) {
                         }
                    }
                } else {
                clock.startTime();
                        if (clock.time(TimeUnit.SECONDS) >= 0 && clock.time(TimeUnit.SECONDS) <= .438) {
                            movingServo.setPower(1);
                        } if (clock.time(TimeUnit.SECONDS) >= .438 && clock.time(TimeUnit.SECONDS) <= 1) {
                            movingServo.setPower(0);
                        } if (clock.time(TimeUnit.SECONDS) >= 6) {
                            clock.reset();
                        }



                }
            telemetry.update();



        }




    }
}









}












            if (gamepad1.b && !clockStarted) {
                clock.reset();
                clockStarted = true;
                movingServo.setPower(1);    // start servo
            }

            if (gamepad1.y && clockStarted) {
                clockStarted = false;
                movingServo.setPower(0);    // stop servo
                finalTime = clock.seconds(); // save frozen time
            }

            if (clockStarted) {
                telemetry.addData("Servo Power", "Running");
                telemetry.addData("Timer (live)", clock.seconds());
            } else {
                telemetry.addData("Servo Power", "Stopped");
                telemetry.addData("Timer (final)", finalTime);
            }

            telemetry.update();

            while (gamepad1.x) {
                movingServo.setPower(1);
            }

            while (gamepad1.y) {
                movingServo.setPower(-1);
            }


ElapsedTime timer = new ElapsedTime();
            boolean timerRunning = false;
            double finalTime = 0;


                 if (gamepad1.y && !timerRunning) {
                    timer.reset();
                    timerRunning = true;
                    movingServo.setPower(1);    // start servo
                }

                 if (gamepad1.x && timerRunning) {
                    timerRunning = false;
                    movingServo.setPower(0);    // stop servo
                    finalTime = timer.seconds(); // save frozen time
                }

                 if (timerRunning) {
                    telemetry.addData("Servo Power", "Running");
                    telemetry.addData("Timer (live)", timer.seconds());
                } else {
                    telemetry.addData("Servo Power", "Stopped");
                    telemetry.addData("Timer (final)", finalTime);
                }

 while (gamepad1.x) {
                movingServo.setPower(1);
            }

            while (gamepad1.y) {
                movingServo.setPower(-1);
            }






*/



