package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


class ENCODER extends OpMode {
    DcMotorEx turretPower;
    static final double tPR = 537.6;
    static final double tRPM = 450;



    @Override
    public void init() {


        turretPower = hardwareMap.get(DcMotorEx.class, "turret_Motor");
        telemetry.addData("Hardware", "Initialized");
        turretPower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    @Override
    public void loop() {

            if (gamepad1.dpad_left) {
                double ticksPerSecond =
                        (tRPM / 60.0) * tPR;


                turretPower.setVelocity(ticksPerSecond);
            }
            else {
                turretPower.setPower(0);
            }


            telemetry.addData("Target RPM", tRPM);
            telemetry.addData("Actual RPM", turretPower.getVelocity() * 60 / tPR);
            telemetry.update();

        }
    }