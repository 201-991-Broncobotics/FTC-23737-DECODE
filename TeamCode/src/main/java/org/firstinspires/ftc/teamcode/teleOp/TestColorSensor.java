package org.firstinspires.ftc.teamcode.teleOp;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


//this code is for getting the RGB values for the 2 artifacts, which will be imputed for the sorting mechanism
public class TestColorSensor {

    public NormalizedColorSensor colorSensor;
    public Telemetry telemetry;
    public double normRed, normGreen, normBlue;



    public void init(HardwareMap hwMap, Telemetry telemetry) {
        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");
        this.telemetry = telemetry;
    }

    public enum DetectedColor {
        GREEN(   .02,    .03,  .06, .7,  .04,   .05),

        PURPLE(   .031,    .05, .04,  .059, .045,   .06),
        UNKNOWN(0, 0, 0, 0, 0, 0);


        private final double redMin, redMax;
        private final double greenMin, greenMax;
        private final double blueMin, blueMax;


        DetectedColor(double redMin, double redMax,
                      double greenMin, double greenMax,
                      double blueMin, double blueMax) {
            this.redMin   = redMin;
            this.redMax   = redMax;
            this.greenMin = greenMin;
            this.greenMax = greenMax;
            this.blueMin  = blueMin;
            this.blueMax  = blueMax;

        }

//this..
    }

    public DetectedColor getColors() {
        colorSensor.setGain(25);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        normGreen = colors.green;
        normRed = colors.red;
        normBlue = colors.blue;

        telemetry.addData("red", normRed);
        telemetry.addData("blue", normBlue);
        telemetry.addData("green", normGreen);


        if (normRed >= DetectedColor.GREEN.redMin && normRed <= DetectedColor.GREEN.redMax &&
                normGreen >= DetectedColor.GREEN.greenMin && normGreen <= DetectedColor.GREEN.greenMax &&
                normBlue >= DetectedColor.GREEN.blueMin && normBlue <= DetectedColor.GREEN.blueMax) {

                telemetry.addData("Matched Color", "GREEN");
                return DetectedColor.GREEN;

        }

         if (normRed   >= DetectedColor.PURPLE.redMin   && normRed   <= DetectedColor.PURPLE.redMax &&
                normGreen >= DetectedColor.PURPLE.greenMin && normGreen <= DetectedColor.PURPLE.greenMax &&
                normBlue  >= DetectedColor.PURPLE.blueMin  && normBlue  <= DetectedColor.PURPLE.blueMax) {

            telemetry.addData("Matched Color", "PURPLE");
            return DetectedColor.PURPLE;
        }


        telemetry.addData("Matched Color", "UNKNOWN");
        return DetectedColor.UNKNOWN;

    }
}