package org.firstinspires.ftc.teamcode.teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Configurable
@TeleOp
public class FlywheelTuner extends OpMode{
    public DcMotorEx Flywheel1;
    public DcMotorEx Flywheel2;

    public double lowVel = 1500;

    public double highVel = 2100;

    double targetVel = highVel;

    double F = 0;
    double P = 0;

    double[] stepSizes = {50.0, 10.0, 1.0, 0.5, 0.05, .005, .0005};

    int stepIndex = 1;



    @Override
    public void init() {
        Flywheel1 = hardwareMap.get(DcMotorEx.class, "flyMotor");
        Flywheel2 = hardwareMap.get(DcMotorEx.class, "flyMotor2");
        Flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        Flywheel1.setDirection(DcMotorEx.Direction.REVERSE);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }


    @Override
    public void loop() {
        //get all our gamepad commands
        //set target velocity
        //update telemetry

        if (gamepad1.yWasPressed()) {
            if (targetVel == highVel) {
                targetVel = lowVel;
            }

            else {
                targetVel = highVel;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }



        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }


        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        Flywheel1.setVelocity(targetVel);
        Flywheel2.setVelocity(targetVel);

        double currentVel = Flywheel2.getVelocity();
        double error = targetVel - currentVel;

        telemetry.addData("Target Velocity", targetVel);
        telemetry.addData("Current Velocity", currentVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("-------------------", 0);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R", F);
        telemetry.addData("Step Size", "%.4f (B Button", stepSizes[stepIndex]);
    }

}
