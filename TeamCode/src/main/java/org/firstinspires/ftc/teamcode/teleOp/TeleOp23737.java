
package org.firstinspires.ftc.teamcode.teleOp;
import static com.pedropathing.math.MathFunctions.clamp;
import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.stream.IntStream;

import dev.nextftc.hardware.impl.ServoEx;




@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends LinearOpMode { //if opmode isn't working, change back to linearopmode. It can be found in the emergancy code
    TestColorSensor colorSensor = new TestColorSensor();
    CRServo movingServo;

    Servo flyWheel;
    DcMotor turnTable, turretPower, motorFlyWheel, motorIntake;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public AnalogInput axonEncoder;
    ElapsedTime clock;
    boolean cycleRunning;



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
        axonEncoder = hardwareMap.get(AnalogInput.class, "encoder"); //TODO: Change name
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
        float x = gamepad1.right_trigger;
        float y = gamepad1.left_trigger;
        TestColorSensor.DetectedColor detected = colorSensor.getColors();
        //double time = clock.seconds();


        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            axonEncoder.getVoltage();
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


            clock = new ElapsedTime();


            if (gamepad1.y) {
                movingServo.setPower(1);
            }
            if (gamepad1.x) {
                movingServo.setPower(.5);
            }
            if (gamepad1.b) {
                movingServo.setPower(0);
            }


             if (axonEncoder.getVoltage() >= 2.6 && axonEncoder.getVoltage() <= 2.8) {
                 movingServo.setPower(0);
                 if (detected == TestColorSensor.DetectedColor.GREEN || detected == TestColorSensor.DetectedColor.PURPLE) {
                     if (!cycleRunning) {
                         clock.reset();
                         cycleRunning = true;
                     }
                     telemetry.addData("Color detected", detected);
                     if (clock.time(TimeUnit.SECONDS) <= 2) {
                         turretPower.setPower(1);
                         motorFlyWheel.setPower(1);
                         flyWheel.setPosition(.37);
                     }  else if (clock.time(TimeUnit.SECONDS) > 2) {
                         movingServo.setPower(.5);
                         motorFlyWheel.setPower(0);
                         turretPower.setPower(0);
                         flyWheel.setPosition(.725);
                         cycleRunning = false;
                     }
                 } else if (!cycleRunning) {
                         clock.reset();
                         cycleRunning = true;
                 } if (clock.time(TimeUnit.SECONDS) <= 2) {
                         movingServo.setPower(0);
                 } else {
                         telemetry.addLine("Couldn't detect color");
                         movingServo.setPower(.5);
                         cycleRunning = false;
                 }

             }

             if (axonEncoder.getVoltage() >= .5 && axonEncoder.getVoltage() <= .7) {
                 movingServo.setPower(0);
                 if (detected == TestColorSensor.DetectedColor.GREEN || detected == TestColorSensor.DetectedColor.PURPLE) {
                     if (!cycleRunning) {
                         clock.reset();
                         cycleRunning = true;
                     }
                     telemetry.addData("Color detected", detected);
                     if (clock.time(TimeUnit.SECONDS) <= 2) {
                         turretPower.setPower(1);
                         motorFlyWheel.setPower(1);
                         flyWheel.setPosition(.37);
                     }  else if (clock.time(TimeUnit.SECONDS) > 2) {
                         movingServo.setPower(.5);
                         motorFlyWheel.setPower(0);
                         turretPower.setPower(0);
                         flyWheel.setPosition(.725);
                         cycleRunning = false;
                     }
                 } else if (!cycleRunning) {
                     clock.reset();
                     cycleRunning = true;
                 } if (clock.time(TimeUnit.SECONDS) <= 2) {
                     movingServo.setPower(0);
                 } else {
                     telemetry.addLine("Couldn't detect color");
                     movingServo.setPower(.5);
                     cycleRunning = false;
                 }


             }

             // Try to see if the code will work for these two. If they do, change the code for the ones that are commented out
             /*

              if (axonEncoder.getVoltage() >= 1.55 && axonEncoder.getVoltage() <= 1.8) {
                movingServo.setPower(0);
                if (detected == TestColorSensor.DetectedColor.GREEN || detected == TestColorSensor.DetectedColor.PURPLE) {
                    if (!cycleRunning) {
                        clock.reset();
                        cycleRunning = true;
                    }
                    telemetry.addData("Color detected", detected);
                    if (clock.time(TimeUnit.SECONDS) <= 2) {
                        turretPower.setPower(1);
                        motorFlyWheel.setPower(1);
                        flyWheel.setPosition(.37);
                    }  else if (clock.time(TimeUnit.SECONDS) > 2) {
                        movingServo.setPower(.1);
                        motorFlyWheel.setPower(0);
                        turretPower.setPower(0);
                        flyWheel.setPosition(.725);
                        cycleRunning = false;
                    }
                } else {
                    telemetry.addLine("Couldn't detect color");
                    movingServo.setPower(.1);
                    cycleRunning = false;
                }

             }

            if (axonEncoder.getVoltage() >= .01 && axonEncoder.getVoltage() <= .04) {
                movingServo.setPower(0);
                if (detected == TestColorSensor.DetectedColor.GREEN || detected == TestColorSensor.DetectedColor.PURPLE) {
                    if (!cycleRunning) {
                        clock.reset();
                        cycleRunning = true;
                    }
                    telemetry.addData("Color detected", detected);
                    if (clock.time(TimeUnit.SECONDS) <= 2) {
                        turretPower.setPower(1);
                        motorFlyWheel.setPower(1);
                        flyWheel.setPosition(.37);
                    }  else if (clock.time(TimeUnit.SECONDS) > 2) {
                        movingServo.setPower(.1);
                        motorFlyWheel.setPower(0);
                        turretPower.setPower(0);
                        flyWheel.setPosition(.725);
                        cycleRunning = false;
                    }
                } else {
                    telemetry.addLine("Couldn't detect color");
                    movingServo.setPower(.1);
                    cycleRunning = false;
                }
            }

            if (axonEncoder.getVoltage() >= 2.15 && axonEncoder.getVoltage() <= 2.35) {movingServo.setPower(0);
                if (detected == TestColorSensor.DetectedColor.GREEN || detected == TestColorSensor.DetectedColor.PURPLE) {
                    if (!cycleRunning) {
                        clock.reset();
                        cycleRunning = true;
                    }
                    telemetry.addData("Color detected", detected);
                    if (clock.time(TimeUnit.SECONDS) <= 2) {
                        turretPower.setPower(1);
                        motorFlyWheel.setPower(1);
                        flyWheel.setPosition(.37);
                    }  else if (clock.time(TimeUnit.SECONDS) > 2) {
                        movingServo.setPower(.1);
                        motorFlyWheel.setPower(0);
                        turretPower.setPower(0);
                        flyWheel.setPosition(.725);
                        cycleRunning = false;
                    }
                } else {
                    telemetry.addLine("Couldn't detect color");
                    movingServo.setPower(.1);
                    cycleRunning = false;
                }
            }

            if (axonEncoder.getVoltage() >= .9 && axonEncoder.getVoltage() <= 1.2) {
                movingServo.setPower(0);
                if (detected == TestColorSensor.DetectedColor.GREEN || detected == TestColorSensor.DetectedColor.PURPLE) {
                    if (!cycleRunning) {
                        clock.reset();
                        cycleRunning = true;
                    }
                    telemetry.addData("Color detected", detected);
                    if (clock.time(TimeUnit.SECONDS) <= 2) {
                        turretPower.setPower(1);
                        motorFlyWheel.setPower(1);
                        flyWheel.setPosition(.37);
                    } else if (clock.time(TimeUnit.SECONDS) > 2) {
                        movingServo.setPower(.5);
                        motorFlyWheel.setPower(0);
                        turretPower.setPower(0);
                        flyWheel.setPosition(.725);
                        cycleRunning = false;
                    }
                } else if (!cycleRunning) {
                    clock.reset();
                    cycleRunning = true;
                }
                if (clock.time(TimeUnit.SECONDS) <= 2) {
                    movingServo.setPower(0);
                } else {
                    telemetry.addLine("Couldn't detect color");
                    movingServo.setPower(.5);
                    cycleRunning = false;
                }


            }

              */



            telemetry.addData("Axon Encoder Voltage: ", axonEncoder.getVoltage());
            telemetry.update();

        }
    }
}
//intake to intake  *2.794    *1.689     *0.6
//shooting to shooting  1.094    *2.257  *.02




















