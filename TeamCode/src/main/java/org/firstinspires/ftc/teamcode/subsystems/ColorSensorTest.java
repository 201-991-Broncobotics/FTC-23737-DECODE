package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ColorSensorTest extends OpMode {
    TestBenchColor bench = new TestBenchColor();
    TestBenchColor.DetectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap);

    }

    @Override
    public void loop() {
       detectedColor = bench.getDectectedColor(telemetry);
       telemetry.addData("Color Dectected", detectedColor);
    }


}
