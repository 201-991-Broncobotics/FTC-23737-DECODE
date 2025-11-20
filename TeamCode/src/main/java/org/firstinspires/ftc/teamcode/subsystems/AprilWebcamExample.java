package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap,telemetry);

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelementry(id20);
    }
}