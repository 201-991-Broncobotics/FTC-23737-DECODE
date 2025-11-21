package org.firstinspires.ftc.teamcode.auton;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auton23737")
public class Auton23737 extends OpMode {

    DcMotor front_Right_Wheel;
    DcMotor back_Right_Wheel;
    DcMotor back_Left_Wheel;
    DcMotor front_Left_Wheel;
    ElapsedTime clock = new ElapsedTime();

    @Override
    public void init() {

        front_Right_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        back_Right_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        back_Left_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        front_Left_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        clock.reset();
        clock.startTime();

    }

    @Override
    public void loop() {

        if (clock.time(TimeUnit.SECONDS) <= 3) {

            front_Right_Wheel.setPower(-1);
            back_Right_Wheel.setPower(-1);
            back_Left_Wheel.setPower(-1);
            front_Left_Wheel.setPower(-1);

        } else if (clock.time(TimeUnit.SECONDS) >= 3 && (clock.time(TimeUnit.SECONDS) <= 4)) {

            front_Right_Wheel.setPower(1);
            back_Right_Wheel.setPower(-1);
            back_Left_Wheel.setPower(1);
            front_Left_Wheel.setPower(-1);

        } else {

            front_Right_Wheel.setPower(0);
            back_Right_Wheel.setPower(0);
            back_Left_Wheel.setPower(0);
            front_Left_Wheel.setPower(0);

        }
    }
}
