package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

        public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
                .forwardPodY(1)
                .strafePodX(1)
                .forwardEncoder_HardwareMapName("leftFront") // change the name and directions once given
                .strafeEncoder_HardwareMapName("rightRear")
                .IMU_HardwareMapName("imu")
                .forwardPodY(2.5)
                .strafePodX(.2)
                .forwardTicksToInches(1)
                .strafeTicksToInches(1)


                .IMU_Orientation(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                );

        public static MecanumConstants driveConstants = new MecanumConstants()
                .yVelocity(1)
                .xVelocity(1)
                .maxPower(1)
                .rightFrontMotorName("fRM")
                .rightRearMotorName("bRM")
                .leftRearMotorName("bLM")
                .leftFrontMotorName("fLM")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);


        public static FollowerConstants followerConstants = new FollowerConstants().mass(11.3)
                .lateralZeroPowerAcceleration(1)
                .forwardZeroPowerAcceleration(1);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


        public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
        }
    }