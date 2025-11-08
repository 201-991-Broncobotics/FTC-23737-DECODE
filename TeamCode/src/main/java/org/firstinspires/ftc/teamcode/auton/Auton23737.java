package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class auton extends OpMode {

    DcMotor front_Right_Wheel;
    DcMotor back_Right_Wheel;
    DcMotor back_Left_Wheel;
    DcMotor front_Left_Wheel;

    @Override
    public void init() {

        front_Right_Wheel.setPower(1);
        back_Right_Wheel.setPower(1);
        back_Left_Wheel.setPower(1);
        front_Left_Wheel.setPower(1);

        front_Right_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        back_Right_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        back_Left_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        front_Left_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {

    }
}
