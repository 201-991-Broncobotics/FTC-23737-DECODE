package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ATT extends OpMode {

    private Limelight3A limelights;
    private IMU imu;

    private static final double tTP = 0.02;// this will change
    private static final double mP = 0.4;
    private static final double sT = 1.0; // degrees
    private double error, tolerance = 3;
    private ElapsedTime searchTimer = new ElapsedTime();

    private boolean sL = true;
    private static final double sP = 0.2;   // slow scan speed
    private static final double wT = 0.75; // seconds before switching direction
    DcMotor turnTable;

    @Override
    public void init() {

        turnTable = hardwareMap.get(DcMotor.class, "turn_Table");
        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setDirection(DcMotorSimple.Direction.REVERSE);
        limelights.start();

    }

    @Override
    public void loop() {

        LLResult llResult = limelights.getLatestResult();
        if (llResult != null && llResult.isValid()) {

            double tx = llResult.getTx();
            double turnPower = tx * tTP;
            error = tx - tolerance;

            if (Math.abs(tx) < sT) {
                turnPower = 0;

            }

            turnPower = Math.max(-mP, Math.min(mP, turnPower));
            turnTable.setPower(turnPower);
            telemetry.addData("tag detected", true);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("turn power", turnPower);

        } else {
            // no tag detected -> look left and right
             if (searchTimer.seconds() > wT) {
                sL = !sL;   // flip direction
                searchTimer.reset();

            }

            double searchPower = sL ? sP : -sP;
            turnTable.setPower(searchPower);
            telemetry.addData("Tag Detected", false);
            telemetry.addData("Searching", sL ? "Left" : "Right");

        }
    }
}
