package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mech.config.dbDeg;
import static org.firstinspires.ftc.teamcode.mech.config.kS;
import static org.firstinspires.ftc.teamcode.mech.config.kffGyro;
import static org.firstinspires.ftc.teamcode.mech.config.kpTurretLL;
import static org.firstinspires.ftc.teamcode.mech.config.maxStaleMs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class turretTest extends LinearOpMode {
    // 0 = tags 30-33, 1 = 34-37, 2 = 38-41, 3 = 42-45
    public static int pipeline = 0;

    @Override
    public void runOpMode() {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        CRServo turret = hardwareMap.get(CRServo.class, "turretServo");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.setMsTransmissionInterval(100);
        ll.setPollRateHz(100);
        ll.pipelineSwitch(pipeline);
        ll.start();

        waitForStart();

        boolean manualControl = false;
        boolean haveTx = false;
        double tx = 0.0;
        double lastFrameTs = -1;

        while (opModeIsActive()) {
            double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            if (gamepad1.rightBumperWasReleased()) manualControl = !manualControl;

            if (manualControl) {
                turret.setPower(gamepad1.right_stick_x);
                continue;
            }

            LLResult r = ll.getLatestResult();

            if (r != null && r.isValid()
                    && r.getPipelineIndex() == pipeline
                    && r.getStaleness() < maxStaleMs) {

                if (r.getTimestamp() != lastFrameTs) {
                    lastFrameTs = r.getTimestamp();
                    double minTx = Double.MAX_VALUE, maxTx = -Double.MAX_VALUE;
                    haveTx = false;
                    for (LLResultTypes.FiducialResult fr : r.getFiducialResults()) {
                        double t = fr.getTargetXDegreesNoCrosshair();
                        if (t < minTx) minTx = t;
                        if (t > maxTx) maxTx = t;
                        haveTx = true;
                    }
                    if (haveTx) tx = (minTx + maxTx) / 2.0;
                }
            } else {
                haveTx = false;
            }

            double power;
            if (haveTx) {
                power = (Math.abs(tx) > dbDeg)
                        ? -(tx * kpTurretLL + Math.copySign(kS, tx)) //+ yawRate * kffGyro)
                        : 0.0;
            } else {
                power = (Math.abs(yawRate) > 5.0)
                        ? yawRate * kffGyro
                        : 0.0;
            }

            turret.setPower(Range.clip(power, -0.6, 0.6));

            telemetry.addData("pipeline", r != null ? r.getPipelineIndex() : -1);
            telemetry.addData("tag", haveTx);
            telemetry.addData("tx", tx);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}