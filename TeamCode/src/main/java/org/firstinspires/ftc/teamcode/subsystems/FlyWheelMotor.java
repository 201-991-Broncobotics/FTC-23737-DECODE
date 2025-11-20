package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FlyWheelMotor extends OpMode {

    DcMotor fWM;
    public double powerLimit;

    @Override
    public void init() {

        fWM = hardwareMap.get(DcMotor.class, "FlyWheelMotor");
        powerLimit = 1;
        fWM.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    @Override
    public void loop() {

        fWM.setPower((gamepad1.right_trigger)* powerLimit);

    }
}