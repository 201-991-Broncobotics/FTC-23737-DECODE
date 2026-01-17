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
        GREEN(/* redMin .012 */   .012, /* redMax .015 */   .015, /* greenMin .03 */ .03, /* greenMax .04*/
                .04,/* blueMin ,023 */  .023, /* blueMax .03 */  .03),

        PURPLE(/* redMin */   .01, /* redMax */   .02,/* greenMin */ .011, /* greenMax */ .015,
                /* blueMin */  .012, /* blueMax */  .016),
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

//this..
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