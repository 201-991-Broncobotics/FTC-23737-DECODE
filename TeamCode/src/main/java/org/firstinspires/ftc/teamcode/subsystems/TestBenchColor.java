package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {
    RevColorSensorV3 colorSensor;

    public enum DetectedColor {
        GREEN,
        BLUE,
        RED,
        YELLOW,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(RevColorSensorV3.class, "sensor_color_distance");
        colorSensor.getGain();
    }

    public DetectedColor getDectectedColor(Telemetry telemerty) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normGreen, normBlue, normRed;
        normBlue = colors.blue / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normRed = colors.red / colors.alpha;

        telemerty.addData("red", normRed);
        telemerty.addData("green", normGreen);
        telemerty.addData("blue", normBlue);

            /*
            red, green, blue
            RED = >.35, <.3, <.3
            YELLOW = >.5, >.9, <.6
            BLUE = <.2, <.5, >.5
            PURPLE =
            GREEN =
             */

        if (normRed > .35 && normGreen < .3 && normBlue < .3) {
            return DetectedColor.RED;
        } else if (normRed > .5 && normGreen > .9 && normBlue < .6) {
            return DetectedColor.YELLOW;
        } else if (normRed < .2 && normGreen < .5 && normBlue > .5) {
            return DetectedColor.BLUE;
        } else if (normRed > .4 && normGreen > .8 && normBlue < .5) {
            return DetectedColor.PURPLE;
        } else if (normRed > .3 && normGreen > .8 && normBlue < .5) {
            return DetectedColor.GREEN;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
}