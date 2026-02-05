























package org.firstinspires.ftc.teamcode.auton;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




import java.util.concurrent.TimeUnit;




@Autonomous(name = "Auton23737")
public class Auton23737 extends OpMode {

    Servo indexer;
    Servo kicky;
    DcMotor turnTable, motorFlyWheel, intake;
    DcMotorEx turretPower, backLeftMotor, backRightMotor;
    Servo flyWheel;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    ElapsedTime clock = new ElapsedTime();

    private Follower follower;
    private Timer pathtime,opmodetime;

    private final Pose startPose = new Pose(123.79076923076924, 122.5353846153846, Math.toRadians(37));
    private final Pose shootingPose = new Pose(84.40615384615386, 83.74153846153847, Math.toRadians(45));
    private final Pose getBall1 = new Pose(125.1476923076923, 83.91076923076925, Math.toRadians(0));
    public enum Pathstates {
        StartPOS_TO_ShootPOS,
        Shoot_Preload,
        ShootPOS_To_getBall1POS
    }

    Pathstates pathstate;

    private PathChain startPOStoshootPOS;
    private PathChain shootPOStogetball1POS;

    public void buildPaths() {
        startPOStoshootPOS = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();
    }

    public void pathUpdate () {
        turretPower.setPower(1);
        switch (pathstate) {
            case StartPOS_TO_ShootPOS:
                follower.followPath(startPOStoshootPOS, true);
                setPathState(Pathstates.Shoot_Preload);
                break;
            case Shoot_Preload:
                if (follower.isBusy()) {
                    indexer.setPosition(.15);
                }
                if (!follower.isBusy()) {
                    motorFlyWheel.setPower(1);
                    turretPower.setPower(1);
                    kicky.setPosition(.32);
                }

            default:
                telemetry.addLine("Auton didn't work");
                break;
        }
    }

    public void setPathState (Pathstates newstate) {
        pathstate = newstate;
        pathtime.resetTimer();
    }

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "fLM");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fRM");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bRM");
        flyWheel = hardwareMap.get(Servo.class, "fly_Wheel");
        turretPower = hardwareMap.get(DcMotorEx.class, "turret_Motor");
        motorFlyWheel = hardwareMap.get(DcMotor.class, "mFly");
        turnTable = hardwareMap.get(DcMotor.class, "turn_Table");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        turretPower.setDirection(DcMotor.Direction.REVERSE);
        motorFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        kicky = hardwareMap.get(Servo.class, "fly_Wheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(Servo.class, "sServo");

        pathstate = Pathstates.StartPOS_TO_ShootPOS;
        pathtime = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();

    }
}
