package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turntable extends OpMode {
    DcMotor turntable;
    public double powerLimit;
    @Override
    public void init() {
        turntable =  hardwareMap.get(DcMotor.class, "turntable");
        powerLimit = .06;
    }

    @Override
    public void loop() {
        turntable.setPower((gamepad1.left_stick_x)*powerLimit);
    }
}