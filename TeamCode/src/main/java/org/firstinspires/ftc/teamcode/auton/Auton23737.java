package org.firstinspires.ftc.teamcode.auton;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auton23737")
public class Auton23737 extends OpMode {

    Servo flyWheel;
    DcMotor turnTable, turretPower, motorFlyWheel;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    ElapsedTime clock = new ElapsedTime();

    @Override
    public void init() {

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "fLM");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "bLM");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "fRM");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "bRM");
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
        clock.reset();
        clock.startTime();

    }

    @Override
    public void loop() {

        if (clock.time(TimeUnit.SECONDS) <= 2) {

            frontRightMotor.setPower(4);
            backRightMotor.setPower(4);
            backLeftMotor.setPower(4);
            frontLeftMotor.setPower(4);
            
        } else if (clock.time(TimeUnit.SECONDS) >= 2 && (clock.time(TimeUnit.SECONDS) <= 3)) {

            frontRightMotor.setPower(1);
            backRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
            frontLeftMotor.setPower(-1);

        } else if (clock.time(TimeUnit.SECONDS) >=3 && (clock.time(TimeUnit.SECONDS) <=4)) {

            flyWheel.setPosition(11);
            motorFlyWheel.setPower(5);
            turretPower.setPower(5);

        } else {

            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);

        }
    }
}
