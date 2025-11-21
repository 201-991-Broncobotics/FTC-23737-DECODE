package org.firstinspires.ftc.teamcode.teleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Required import
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends LinearOpMode {
    TestColorSensor colorSensor = new TestColorSensor();
    Servo movingServo;
    Servo flyWheel;
    DcMotor turnTable, turretPower, motorFlyWheel;


    public void runOpMode() {


        //private ElapsedTime runtime = new ElapsedTime();
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "fLM");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "bLM");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "fRM");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "bRM");
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
        turnTable.setDirection(DcMotorSimple.Direction.FORWARD);
        colorSensor.init(hardwareMap, telemetry);


        double max, forward, strafe, rotate, throttle, magnitude, angle, frontLeftPower, frontRightPower, backLeftPower, backRightPower;


        waitForStart();


        while (opModeIsActive() && !isStopRequested()) { // idk if this way of making an OpMode might fix the issue


            colorSensor.getColors();


            // Might slow down code a bit by declaring a new variable every time, so I moved it so it declares each variable once ^ at the start


            forward = (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            strafe = (gamepad1.left_stick_x);
            rotate = (gamepad1.right_stick_x);
            throttle = 0.8 + 0.2 * gamepad1.left_trigger; // must equal 1, easy way to create a boost button


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


            frontLeftMotor.setPower(frontLeftPower * throttle);
            frontRightMotor.setPower(frontRightPower * throttle);
            backLeftMotor.setPower(backLeftPower * throttle);
            backRightMotor.setPower(backRightPower * throttle);


            float x = gamepad1.right_trigger;
            float y = gamepad1.left_trigger;


            if (x > 0) {
                turnTable.setPower(x);
            }
            telemetry.addData("power of motor", turnTable);
            telemetry.update();
            turnTable.setPower(0);


            if (y > 0) {
                turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
                turnTable.setPower(-y);
            }
            turnTable.setPower(0);


            if (gamepad1.dpad_up) {
                flyWheel.setPosition(0);
                motorFlyWheel.setPower(-1);
            } else if (!gamepad1.dpad_up) {
                flyWheel.setPosition(.05);
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
                movingServo.setPosition(.9);//  0.85 and 0.25 works
            }
            if (gamepad1.b) {
                movingServo.setPosition(.30); // .65 .45 and .05 (One of the pair will share 0.85)
            }
            if (gamepad1.x) {
                movingServo.setPosition(0.50); // .65 and .1 works


            }
            if (gamepad1.y) {
                movingServo.setPosition(1);
            }
            telemetry.update();


        }
    }
}



         /* if(gamepad1.y) {
               TestColorSensor.DetectedColor detected = colorSensor.getColors();


               if (detected == TestColorSensor.DetectedColor.GREEN) {
                   movingServo.setPosition(1.0);  // example position
               }
               else if (detected == TestColorSensor.DetectedColor.PURPLE) {
                   movingServo.setPosition(0.0);  // example position
               }
               else {  // UNKNOWN
                   movingServo.setPosition(0.5);  // example fallback action
               }




           }

          */
