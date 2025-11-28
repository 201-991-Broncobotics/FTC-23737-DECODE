
package org.firstinspires.ftc.teamcode.teleOp;
import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Required import
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//brush

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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


@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends LinearOpMode { //if opmode isn't working, change back to linearopmode. It can be found in the emergancy code
    TestColorSensor colorSensor = new TestColorSensor();
    Servo movingServo;
    Servo flyWheel;
    DcMotor turnTable, turretPower, motorFlyWheel;
    private ElapsedTime clock = new ElapsedTime();
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;


    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fLM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fRM");
        backRightMotor = hardwareMap.get(DcMotor.class, "bRM");
        movingServo = hardwareMap.get(Servo.class, "sorting_Servo");
        flyWheel = hardwareMap.get(Servo.class, "fly_Wheel");
        turretPower = hardwareMap.get(DcMotor.class, "turret_Motor");
        motorFlyWheel = hardwareMap.get(DcMotor.class, "mFly");
        turnTable = hardwareMap.get(DcMotor.class, "turn_Table");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        motorFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        turnTable.setDirection(DcMotorSimple.Direction.FORWARD);
        colorSensor.init(hardwareMap, telemetry);
        colorSensor.getColors();


        waitForStart();

        while (opModeIsActive()) {

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
//            frontLeftMotor.setPower(frontLeftPower * throttle);
//            frontRightMotor.setPower(frontRightPower * throttle);
//            backLeftMotor.setPower(backLeftPower * throttle);
//            backRightMotor.setPower(backRightPower * throttle);


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

            if (gamepad1.a) {
                //\ movingServo.setDirection(DcMotorSimple.Direction.FORWARD);
                //movingServo.setPower(.5);
                sleep(500);
                //  movingServo.setPower(0);
                // movingServo.setPosition(0.67);    //  0.85 (intake)and 0.67 (shooting)works

            }
            if (gamepad1.b) {

                // movingServo.setDirection(DcMotorSimple.Direction.REVERSE);
                //  movingServo.setPower(.5);
                sleep(500);
                // movingServo.setPower(0);
                // movingServo.setPosition(0.1);            // 0.25(intake) and .1(shooting)
            }
            if (gamepad1.x) {

                // movingServo.setDirection(DcMotorSimple.Direction.FORWARD);
                // movingServo.setPower(1);
                sleep(500);
                // movingServo.setPower(0);
                // movingServo.setPosition(0.85);

            }
            if (gamepad1.y) {

                // movingServo.setDirection(DcMotorSimple.Direction.FORWARD);
                // movingServo.setPower(.1);
                sleep(100);
                // movingServo.setPower(0);
                //movingServo.setPosition(.25);
            }


            TestColorSensor.DetectedColor detected = colorSensor.getColors();
            if (detected == TestColorSensor.DetectedColor.GREEN) {
                turretPower.setPower(1);
                flyWheel.setPosition(0.735);
                motorFlyWheel.setPower(1);
                sleep(10000);
                turretPower.setPower(0);
                flyWheel.setPosition(.37);
                motorFlyWheel.setPower(0);
                movingServo.setPosition(.67);
            } else if (detected == TestColorSensor.DetectedColor.PURPLE) {
                turretPower.setPower(1);
                flyWheel.setPosition(0.735);
                motorFlyWheel.setPower(1);
                sleep(10000);
                turretPower.setPower(0);
                flyWheel.setPosition(.37);
                motorFlyWheel.setPower(0);
                movingServo.setPosition(.85);
            } else {
                movingServo.setPosition(.85);
                sleep(1500);
                movingServo.setPosition(.1);
                sleep(1500);
                movingServo.setPosition(.25);
                sleep(1500);
                movingServo.setPosition(.67);
                sleep(1500);


                if (gamepad1.a) {

                }


                telemetry.addData("power of motor", turnTable);
                telemetry.addData("Flywheel Servo Position: ", flyWheel.getPosition());
                telemetry.addData("Flywheel Servo Position: ", flyWheel.getPosition());
                colorSensor.getColors();
                telemetry.update();

            }

            // UNKNOWN

            telemetry.addData("power of motor", turnTable);
            telemetry.addData("Flywheel Servo Position: ", flyWheel.getPosition());
            telemetry.addData("Flywheel Servo Position: ", flyWheel.getPosition());
            colorSensor.getColors();
            telemetry.update();
        }


    }
}



















