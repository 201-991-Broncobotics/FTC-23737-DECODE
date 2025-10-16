package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends OpMode {

    public Gamepad driver, operator;
    public double forward, strafe, powerLimit;
    public DcMotor rsm1, rsm2, lsm1, lsm2, intake;

    @Override
    public void init() {

        driver = gamepad1;
        operator = gamepad2;

        powerLimit = 0.8;

        rsm1 = hardwareMap.get(DcMotor.class, "rsm1");
        rsm2 = hardwareMap.get(DcMotor.class, "rsm2");
        lsm1 = hardwareMap.get(DcMotor.class, "lsm1");
        lsm2 = hardwareMap.get(DcMotor.class, "lsm2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        rsm1.setDirection(DcMotorSimple.Direction.FORWARD);
        rsm2.setDirection(DcMotorSimple.Direction.FORWARD);
        lsm1.setDirection(DcMotorSimple.Direction.FORWARD);
        lsm2.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        rsm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rsm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lsm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lsm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rsm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rsm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lsm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lsm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        forward = driver.left_stick_y;
        strafe = driver.left_stick_x;

        rsm1.setPower((forward - strafe) * powerLimit);
        rsm2.setPower((forward - strafe) * powerLimit);
        lsm1.setPower((-forward + strafe) * powerLimit);
        lsm2.setPower((-forward + strafe) * powerLimit);

        if (operator.y) intake.setPower(0.2);
        else if (!operator.y) intake.setPower(0);
         else if (operator.y && operator.dpad_up) intake.setPower(-0.2);













    }
}
