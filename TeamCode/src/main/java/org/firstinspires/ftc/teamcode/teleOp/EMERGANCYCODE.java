package org.firstinspires.ftc.teamcode.teleOp;

public class EMERGANCYCODE {
}

/*

package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//work?}
/*
package org.firstinspires.ftc.teamcode.teleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Required import
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
public class TeleOp23737 extends LinearOpMode {
    TestColorSensor colorSensor = new TestColorSensor();
    Servo movingServo;
    Servo flyWheel;
    DcMotor turnTable;
    DcMotor turretPower;
    DcMotor motorFlyWheel;
    //private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    ElapsedTime Clock = new ElapsedTime();


    public void runOpMode() throws InterruptedException {

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
        motorFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel.setDirection(Servo.Direction.REVERSE);
        movingServo.setDirection(Servo.Direction.FORWARD);

        colorSensor.init(hardwareMap);

        double max, forward, strafe, rotate, throttle, magnitude, angle, frontLeftPower, frontRightPower, backLeftPower, backRightPower;


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) { // idk if this way of making an OpMode might fix the issue

            // Might slow down code a bit by declaring a new variable every time, so I moved it so it declares each variable once ^ at the start


            forward = (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            strafe = (gamepad1.left_stick_x);
            rotate = (gamepad1.right_stick_x);
            throttle = 0.6 + 0.4 * gamepad1.left_trigger; // must equal 1, easy way to create a boost button

            // One way of making driving smoother
            magnitude = Math.hypot(forward, strafe);
            angle = Math.atan2(forward, strafe);
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

            frontLeftMotor.setPower(frontLeftPower * throttle);
            frontRightMotor.setPower(frontRightPower * throttle);
            backLeftMotor.setPower(backLeftPower * throttle);
            backRightMotor.setPower(backRightPower * throttle);


            float x = gamepad1.right_trigger;

            if (x > 0) {
                turnTable.setPower(x);
            }
            turnTable.setPower(0);


            if (gamepad1.dpad_up) {
                flyWheel.setPosition(0);
                motorFlyWheel.setPower(-1);
            } else if (!gamepad1.dpad_up){
                flyWheel.setPosition(0.5);
                motorFlyWheel.setPower(0);
            }

            if (gamepad1.dpad_left) {
                turretPower.setDirection(DcMotorSimple.Direction.REVERSE);
                turretPower.setPower(0.8);
            }

           if (gamepad1.a) {
                movingServo.setPosition(1); //all the numbers are not the exact ones. I will get the accurate numbers once the ball is created. It is a placeholder.
              // movingServo.wait(10);
            }
           if (gamepad1.b){
               movingServo.setPosition(0.5);
           }

           if(gamepad1.x) {
                TestColorSensor.DetectedColor detected = colorSensor.getColors(telemetry);

                if (detected == TestColorSensor.DetectedColor.GREEN) {
                    movingServo.setPosition(1.0);  // example position
                }
                else if (detected == TestColorSensor.DetectedColor.PURPLE) {
                    movingServo.setPosition(0.0);  // example position
                }
                else {  // UNKNOWN
                    movingServo.setPosition(0.5);  // example fallback action
                }
                 telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
               telemetry.addData("Gamepad Input: ", gamepad1.left_stick_y);
               telemetry.addData("Servo Position: ", movingServo.getPosition());
               telemetry.update();
            }





        }
    }
}

 */



