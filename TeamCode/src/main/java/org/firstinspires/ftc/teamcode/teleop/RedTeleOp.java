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
        ElapsedTime myStopwatch = new ElapsedTime();
        double time = 0;
        double liftTime = 6;

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
        int topColor = topSensor.hasBall();
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
            topColor = topSensor.hasBall();
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
            if (gamepad1.left_trigger > 0.1) {
                driveSpeed = 0.4;
            }
            else {
                driveSpeed = 1;
            }
            // Intake Eject
            if (gamepad2.yWasPressed()) { abState = 2; }
            if (gamepad2.yWasReleased()) { abState = 0; }
            // Preshoot
            if (gamepad2.bWasPressed()) { arState = 3; }
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
                    actSeq = 2;
                    shooter.setVelocity(0);
                    abState = 3;
                    tilt.tiltSetPower(servoPower);
                    break;
                case(3):
                    Turret.tracking = true;
                    shooter.setVelocity(shooterTargetVel);
                    break;
                default:
                    arState = 0;
            }

            //Abhay States
            switch (abState) {
                case(0): //Intake Off
                    actSeq = 1;
                    intake.setAllPower(0);
                    break;
                case(1): //Intake On
                    actSeq = 0;
                    intake.setAllPower(1);
                    if (ballCount >= 2 && arState != 1) {
                        intake.setRollerPower(0);
                    }
                    if (ballCount >= 3 && arState != 1) {
                        abState = 0;
                    }
                    break;
                case(2):
                    actSeq = 1;
                    intake.setAllPower(-0.75);
                    break;
                case(3): //Tilt
                    intake.setAllPower(0);
                    break;
                default:
                    abState = 0;
            }
            //Light States
            switch(actSeq) {
                case(0):
                    signal.setPosition(lightSequences[0][ballCount]);
                    break;
                case(1):
                    switch(topColor) {
                        case(0):
                            signal.setPosition(0.41);
                            break;
                        case(1):
                            if (topSensor.getColor() == "GREEN"){ signal.setPosition(0.52); }
                            else {signal.setPosition(0.7);}
                            break;
                        default:
                            topColor=0;
                        }
                    break;
                case(2):
                    signal.setPosition(0.62);
                    signal.setPosition(myStopwatch.time() >= 0.5 && signal.getLEDColor() == 0.62 && time < liftTime ? 0.28 : 0.62);
                    if (myStopwatch.time() >= 1 || time >= liftTime) {
                        time += myStopwatch.time();
                        myStopwatch.reset();
                    }
                    if (time >= liftTime)
                        signal.setPosition(0.52);
                    break;
                default:
                    actSeq=1;
            }







            //  ---  UPDATES ---
            drive.update(-gamepad1.left_stick_x * driveSpeed, -gamepad1.left_stick_y * driveSpeed, -gamepad1.right_stick_x * driveSpeed);
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



            //Telemetry
            telemetryM.addLine("SHOOTER:");
            telemetryM.addData("Shooter vel", shooter.getVelocity());
            telemetryM.addData("Shooter target vel", shooter.getTargetVelocity());
            telemetryM.addData("Keep Shooter Running", keepShooterRunning);
            telemetryM.addData("Preshoot", preshoot);
            telemetryM.addData("closeB", Mortar.closeB);
            telemetryM.addData("farB", Mortar.farB);
            telemetryM.addLine("");
            telemetryM.addLine("POSE:");
            telemetryM.addData("pose x", pose.getX());
            telemetryM.addData("pose y", pose.getY());
            telemetryM.addData("pose heading", Math.toDegrees(pose.getHeading()));
            telemetryM.addLine("");
            telemetryM.addLine("TURRET:");
            telemetryM.addData("Turret Heading relative", turret.getTurretHeadingRelative());
            telemetryM.addData("Turret target", turret.getTurretHeading());
            telemetryM.addData("Turret Manual Override", turretOverride);
            telemetryM.addLine("");
            telemetryM.addLine("MISC:");
            telemetryM.addData("Ball Count", ballCount);
            telemetryM.addData("hood angle", hood.getHoodPosition());
            telemetryM.addData("LED Color", signal.getLEDColor());
            telemetryM.addData("Rail Position", rail.getPosition());
            telemetryM.addData("Intaking?", intaking);
            telemetryM.addLine("");
            telemetryM.addLine("COLOR SENSOR");
            telemetryM.addData("Bottom Sensor Color", bottomSensor.getColor());
            telemetryM.addData("Middle Sensor Color", middleSensor.getColor());
            telemetryM.addData("Top Sensor Color", topSensor.getColor());


            telemetry.addLine("SHOOTER:");
            telemetry.addData("Shooter vel", shooter.getVelocity());
            telemetry.addData("Shooter target vel", shooter.getTargetVelocity());
            telemetry.addData("Keep Shooter Running", keepShooterRunning);
            telemetry.addData("Preshoot", preshoot);
            telemetry.addData("closeB", Mortar.closeB);
            telemetry.addData("farB", Mortar.farB);
            telemetry.addLine("");
            telemetry.addLine("POSE:");
            telemetry.addData("pose x", pose.getX());
            telemetry.addData("pose y", pose.getY());
            telemetry.addData("pose heading", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("");
            telemetry.addLine("TURRET:");
            telemetry.addData("Turret Heading relative", turret.getTurretHeadingRelative());
            telemetry.addData("Turret target", turret.getTurretHeading());
            telemetry.addData("Turret Manual Override", turretOverride);
            telemetry.addLine("");
            telemetry.addLine("MISC:");
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("hood angle", hood.getHoodPosition());
            telemetry.addData("LED Color", signal.getLEDColor());
            telemetry.addData("Rail Position", rail.getPosition());
            telemetry.addData("Intaking?", intaking);
            telemetry.addLine("");
            telemetry.addLine("COLOR SENSOR");
            telemetry.addData("Bottom Sensor Color", bottomSensor.getColor());
            telemetry.addData("Middle Sensor Color", middleSensor.getColor());
            telemetry.addData("Top Sensor Color", topSensor.getColor());

            telemetryM.update();
            telemetry.update();

        }
    }

}
