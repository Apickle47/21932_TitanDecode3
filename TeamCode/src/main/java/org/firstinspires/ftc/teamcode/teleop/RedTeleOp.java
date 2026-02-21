package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mortar;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.Signal;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Util;

import java.util.Objects;

@Configurable
@TeleOp(name = "RedTeleOp")
public class RedTeleOp extends LinearOpMode {



    public static double reverseIntakeSpeed = -.75;
    public static Pose resetPose = new Pose(72,72,Math.toRadians(270) );

    @Override
    public void runOpMode() throws InterruptedException {

        Util util = new Util();

        Drivetrain drive = new Drivetrain(hardwareMap, util.deviceConf);

        Intake intake = new Intake(hardwareMap, util.deviceConf);
        Turret turret = new Turret(hardwareMap, util.deviceConf, new Pose(72, 72, (3*Math.PI)/2));
        Mortar shooter = new Mortar(hardwareMap, util.deviceConf);
        BottomSensor bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        MiddleSensor middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        TopSensor topSensor = new TopSensor(hardwareMap, util.deviceConf);
        Gate gate = new Gate(hardwareMap, util.deviceConf);
        Hood hood = new Hood(hardwareMap, util.deviceConf, new Pose(72, 72, (3*Math.PI)/2));
        Rail rail = new Rail(hardwareMap, util.deviceConf);
        Signal signal = new Signal(hardwareMap, util.deviceConf);
        Tilt tilt = new Tilt(hardwareMap, util.deviceConf);
        Follower follower = Constants.createFollower(hardwareMap);
        Pose pose;
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));
        follower.update();
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        turret.setBasketPos(Turret.redBasket);

        //sensor.setLEDBrightness(brightness);

        waitForStart();

        follower.startTeleOpDrive(true);
        hood.setHoodPosition(0.6);
        rail.setPosition(Rail.INDEX);
        boolean shooting = false, turretOverride = false, intaking = false, metDistanceSensorThresh = false, keepShooterRunning = true, preshoot = false, manualKicker = false;
        int shooterTargetVel = 0;
        int ballCount = 0;
        Turret.tracking = false;
        int arState = 0;
        int abState = 0;
        double driveSpeed = 1.0;
        boolean wasIntaking = false;
        int servoPower = 0;
        int[] powers = {-1, 0, 1};
        int count = 1;
        int incCount = 0;
        double[] intakeLightSequence = {1.0, 0.29, 0.35, 0.62};
        double[] StorageLightSequence = {1.0, 0.71, 0.5};
        double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));

        // Add more light sequences here if needed

        double[][] lightSequences = {intakeLightSequence, StorageLightSequence};
        int actSeq = 1;

        while(opModeIsActive()) {
            //Constant update variables
            ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
            pose = turret.getPose();
            hood.hoodIncrement(0.05 * incCount, goalDist >= 115 ? Hood.closeHood : Hood.farHood);
            goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
            shooterTargetVel = shooter.calcVelocity(goalDist);


            //  ---  CONTROLS  ---

            // Shooting On/Off
            if (gamepad1.rightBumperWasPressed()) {
                if (arState == 1) {
                    arState = 0;
                }
                else {
                    arState = 1;
                }
            }
            // Intake
            if (gamepad2.rightBumperWasPressed()) { abState = 1; }
            if (gamepad2.leftBumperWasPressed()) { abState = 0; }
            // Tilt
            if (gamepad1.xWasPressed()) {
                arState = 2;
                count += 1;
                if (count > 2) {
                    count = 0;
                }
                servoPower = powers[count];
            }
            // Reset Position
            if (gamepad1.y && gamepad1.dpad_left) {
                turret.resetRobotPose(resetPose);
            }
            // Slow Mode
            if (gamepad1.left_trigger > 0) {
                driveSpeed = 0.4;
            }
            // Intake Eject
            if (gamepad2.yWasPressed()) { abState = 2; }
            if (gamepad2.yWasReleased()) { abState = 0; }
            // Preshoot
            if (gamepad2.bWasPressed()) { abState = 3; }
            // Shooter Speed Override
            if (gamepad1.dpadLeftWasPressed()) {
                Mortar.closeB -= 50;
                Mortar.farB -= 50;
            }
            if (gamepad1.dpadRightWasPressed()) {
                Mortar.closeB += 50;
                Mortar.farB += 50;
            }
            // Hood Angle Override
            if (gamepad1.dpadUpWasPressed()) {
                incCount -= 1;
            }
            if (gamepad1.dpadDownWasPressed()) {
                incCount += 1;
            }




            //   ---  STATES  ---
            // Arush States
            switch (arState) {
                case(0): //Not Shooting
                    gate.setPosition(Gate.CLOSE);
                    Turret.tracking = false;
                    shooter.setVelocity(Mortar.WAIT);
                    break;
                case(1): //Shooting
                    shooter.setVelocity(shooterTargetVel);
                    Turret.tracking = true;
                    if (shooter.safeToShoot(shooterTargetVel)) {
                        intake.setAllPower(1);
                        gate.setPosition(Gate.OPEN);
                    }
                    abState = 1;
                    break;

                case(2):
                    tilt.tiltSetPower(servoPower);
                    break;
                default:
                    arState = 0;
            }

            //Abhay States
            switch (abState) {
                case(0): //Intake Off
                    intake.setAllPower(0);
                    break;
                case(1): //Intake On
                    intake.setAllPower(1);
                    if (ballCount >= 2) {
                        intake.setAllPower(0);
                        intake.setIntakePower(0.75);
                    }
                    break;
                case(2):
                    intake.setAllPower(-0.75);
                    break;
                case(3):
                    Turret.tracking = true;
                    shooter.setVelocity(shooterTargetVel);

            }
            //Light States
            switch(actSeq) {
                case(0):
                    signal.setPosition(lightSequences[0][ballCount]);
                case(1):

            }







            //  ---  UPDATES ---
            drive.update(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, -gamepad1.right_stick_x * driveSpeed);
            intake.update();
            turret.update();
            shooter.update();
            hood.update();
            gate.update();
            rail.update();
            bottomSensor.update();
            middleSensor.update();
            topSensor.update();
            follower.update();


        }
    }

}
