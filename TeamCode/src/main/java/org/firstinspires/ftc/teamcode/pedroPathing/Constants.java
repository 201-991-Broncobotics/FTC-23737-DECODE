package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static OTOSConstants localizerConstants1 = new OTOSConstants()
            .linearScalar(67)
            .hardwareMapName("OTOS")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .turnTicksToInches(6)
            .strafeTicksToInches(7)
            .forwardTicksToInches(67)
            .robotWidth(67)
            .robotLength(67)
            .rightFrontMotorName("rsm1")
            .rightRearMotorName("rsm2")
            .leftRearMotorName("lsm1")
            .leftFrontMotorName("lsm2")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(67)
            .rightFrontMotorName("rsm1")
            .rightRearMotorName("rsm2")
            .leftRearMotorName("lsm1")
            .leftFrontMotorName("lsm2")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static FollowerConstants followerConstants = new FollowerConstants().mass(1);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants1)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
}