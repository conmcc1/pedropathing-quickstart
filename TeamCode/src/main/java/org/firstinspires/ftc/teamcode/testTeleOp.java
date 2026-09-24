package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

import java.util.List;

@TeleOp
public class testTeleOp extends LinearOpMode {
    Limelight3A LL3A;


    private DcMotorEx shooter;
    private CRServo turretServo;
    private DcMotorEx intake;

    private double targetRPM = 4000;
    private double projectedRPM;

    private double KP_TURRET = .01;

    private Follower follower;

    private double triggerPower;

    private int pipeline;

    enum Team{blue, red}

    Team currentTeam = Team.blue;

    Timer profileTimer;


    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: set up pipeline switching, have turret adjust yaw to face hive, toggle limelight aiming, track the apriltags

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        LL3A = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");


        follower = Constants.create(hardwareMap);

        LL3A.start();
        waitForStart();
        while (opModeIsActive()) {

            projectedRPM = motion_profile(targetRPM / 3, targetRPM, profileTimer.seconds());
            shooter.setVelocity((projectedRPM / 60) * 28);

        LLResult result = LL3A.getLatestResult();

        //set drive powers
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        follower.manual(forward, lateral, turn);
        follower.update();


        /*

        //-- RED TEAM --//

        LL3A.pipelineSwitch(0); // IDs 30 31 32 33 on the red cell opposite the field audience
        LL3A.pipelineSwitch(1); // IDs 34 35 36 37 on the red cell audience side

        //-- BLUE TEAM --//

        LL3A.pipelineSwitch(2); // IDs 38 39 40 41 on the blue cell audience side
        LL3A.pipelineSwitch(3); // IDs 42 43 44 45 on the blue cell opposite the audience


          */

        //---------//

        if (gamepad1.dpad_up){
            if (currentTeam == Team.blue){
                currentTeam = Team.red;
            } else {
                currentTeam = Team.blue;
            }
        }



        // LEFT ARROW: AUDIENCE SIDE IS UP

        if (gamepad1.dpad_left) {
            if (currentTeam == Team.blue) {
                LL3A.pipelineSwitch(2); // IDs 38 39 40 41
            } else {
                LL3A.pipelineSwitch(1); // IDs 34 35 36 37
            }
        }


        // RIGHT ARROW: AUDIENCE SIDE IS DOWN

        if (gamepad1.dpad_right) {
            if (currentTeam == Team.blue) {
                LL3A.pipelineSwitch(3); // IDs 42 43 44 45
            } else {
                LL3A.pipelineSwitch(0); // IDs 30 31 32 33
            }
        }


        triggerPower = gamepad1.right_trigger;

        intake.setPower(-triggerPower);

        /*if (result.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                double minTx = Double.MAX_VALUE;
                double maxTx = -Double.MAX_VALUE;

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    double tx = fr.getTargetXDegreesNoCrosshair(); // Horizontal angle offset to the tag
                    if (tx < minTx) minTx = tx;
                    if (tx > maxTx) maxTx = tx;
                }

                // The midpoint offset in degrees relative to the camera center
                double midpointTx = (minTx + maxTx) / 2.0;

                telemetry.addData("midpoint", midpointTx);

                // Use midpointTx to drive your turret motor PID loop
                double turretPower = midpointTx * KP_TURRET;
                turretServo.setPower(turretPower);
            }

        }*/


        telemetry.addData("projectedRPM", projectedRPM);
        telemetry.addData("actual RPM", shooter.getVelocity());
        telemetry.addData("shooter.getPIDFCoefficients", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));


        telemetry.update();
        }
    }

    double motion_profile(double maxAcceleration, double maxVelocity, double elapsed_time) {
        double acceleration_dt = maxVelocity / maxAcceleration;

        if (elapsed_time < acceleration_dt) return maxAcceleration * elapsed_time;

        return maxVelocity;
    }

}
