package org.firstinspires.ftc.teamcode;

/*import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp23737")
public class TeleOp extends OpMode {
    public CRServo PTOServo;

    public ColorSensor colorSensor;

    GamepadEx operator;
    public ElapsedTime timeCheck = new ElapsedTime();

    @Override
    public void init() {

        operator = new GamepadEx(gamepad1);
        PTOServo = hardwareMap.get(CRServo.class, "PTOServo");
        PTOServo.setRunMode(Motor.RunMode.RawPower);

        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        colorSensor.enableLed(true);



    }

    @Override
    public void loop() {

        if (operator.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            timeCheck.reset();
            timeCheck.startTime();
            PTOServo.set(0.2);
            if (timeCheck.time(TimeUnit.SECONDS) > 0.1){
                PTOServo.set(0);
            }
        }

        if (colorSensor.green() > 90 && colorSensor.green() < 200){
            telemetry.addData("Green Detected: ", colorSensor.green());

        } if (colorSensor.red() > 60 && colorSensor.blue() > 80){
            telemetry.addData("Purple Detected: ", colorSensor.green());
             
        }




    }
}
*/
