package org.firstinspires.ftc.teamcode.teleOp;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
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
        GREEN(/* redMin */   .37, /* redMax */   .3819, /* greenMin */ .94, /* greenMax */
                .956,/* blueMin */  .941, /* blueMax */  .943),

        PURPLE(/* redMin */   .9871, /* redMax */   .9875,/* greenMin */ .9871, /* greenMax */ .9875,
                /* blueMin */  .9871, /* blueMax */  .9875),
        UNKNOWN(
                0, 0, 0, 0, 0, 0
        );


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

//this
    }

    public DetectedColor getColors() {
        colorSensor.setGain(60);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        normGreen = colors.green * colors.alpha;
        normRed = colors.red * colors.alpha;
        normBlue = colors.blue * colors.alpha;

        telemetry.addLine("color detected");
        telemetry.addData("red", normRed);
        telemetry.addData("blue", normBlue);
        telemetry.addData("green", normGreen);

        return null;
    }
}