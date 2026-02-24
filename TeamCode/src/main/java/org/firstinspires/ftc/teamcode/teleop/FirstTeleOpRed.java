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
//@TeleOp(name = "FirstTeleOpRed")
public class FirstTeleOpRed extends LinearOpMode {

    //SENSOR
    public static int sensorThresh = 20, brightness = 50;
    //
    public static boolean indexing, tilting;
    public static double reverseIntakeSpeed = -.75;
    public static int maxTurretChange = 10;
    public static int kickerWaitTime = 500;
    public static Pose resetPose = new Pose(72,72,Math.toRadians(270) );



    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        rail.setPosition(Rail.INLINE);
        boolean shooting = false, turretOverride = false, intaking = false, metDistanceSensorThresh = false, keepShooterRunning = true, preshoot = false, manualKicker = false;
        int shooterTargetSpeed = 0;
        int ballCount = 0;
        Turret.tracking = false;



        while(opModeIsActive()) {
            ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
// SHOOTER
            pose = turret.getPose();
            shooterTargetSpeed = shooter.calcVelocity(Math.sqrt(
                    (turret.distanceToBasket().getX() * turret.distanceToBasket().getX()) + (turret.distanceToBasket().getY() * turret.distanceToBasket().getY())));

            if (gamepad1.right_bumper) {
                shooting = true;
            }

            if (gamepad2.bWasPressed()) {
                preshoot = !preshoot;
            }

            if (shooting || preshoot) {
                if (!turretOverride) {
                    Turret.tracking = true;
                }
                shooter.setVelocity(shooterTargetSpeed);

                if (shooting && shooter.getVelocity() > shooterTargetSpeed - Mortar.THRESH && shooter.getVelocity() < shooterTargetSpeed + Mortar.THRESH && shooter.getVelocity() <= shooterTargetSpeed) {
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);
                }
            }

            if (gamepad1.left_bumper) {
                Turret.tracking = false;
                shooting = false;
                preshoot = false;
                gate.setPosition(Gate.CLOSE);
            }

            if (!shooting && !preshoot) {
                if (keepShooterRunning) {
                    shooter.setVelocity(Mortar.WAIT);
                } else {
                    shooter.setVelocity(0);
                }
            }

            if (gamepad1.dpadDownWasPressed()) {
                Mortar.closeB -= 50;
                Mortar.farB -= 50;
            }

            if (gamepad1.dpadUpWasPressed()) {
                Mortar.closeB += 50;
                Mortar.farB += 50;
            }
            if (gamepad1.aWasPressed()) {
                if (rail.getPosition() == Rail.INLINE) {
                    rail.setPosition(Rail.INDEX);
                } else if (rail.getPosition() == Rail.INDEX) {
                    rail.setPosition(Rail.INLINE);
                }
            }
            if (gamepad2.aWasPressed()) {
                keepShooterRunning = !keepShooterRunning;
            }


// INTAKE
            if (gamepad2.right_bumper) {
                intaking = true;
                if (!shooting) {
                    gate.setPosition(Gate.CLOSE);
                }
            }
            if (gamepad2.yWasPressed()) {
                intake.setAllPower(reverseIntakeSpeed);
            }
            if (gamepad2.yWasReleased()) {
                intake.setAllPower(0);
            }

            if (gamepad2.left_bumper) {
                intaking = false;
                intake.setAllPower(0);
            }

            if (gamepad2.dpadRightWasPressed()) {
                hood.hoodIncrement(0.05, hood.getHoodPosition());
            }
            if (gamepad2.dpadLeftWasPressed()) {
                hood.hoodIncrement(-0.05, hood.getHoodPosition());
            }

// SENSOR
            if (intaking & !shooting) {
                if (ballCount == 0) {
                    signal.setPosition(Signal.VIOLET);
                    if (!gamepad2.yWasPressed()) {
                        intake.setAllPower(1);
                    }
                } else if (ballCount == 1) {
                    signal.setPosition(Signal.RED);
                    if (!gamepad2.yWasPressed()) {
                        intake.setAllPower(1);
                    }
                } else if (ballCount == 2) {
                    signal.setPosition(Signal.YELLOW);
                    if (!gamepad2.yWasPressed()){
                        intake.setRollerPower(0);
                        intake.setIntakePower(1);
                    }
                } else if (ballCount == 3) {
                    signal.setPosition(Signal.GREEN);
                    if (!gamepad2.yWasPressed()) {
                        intake.setAllPower(0);
                    }
                    intaking = false;
                }
            }
            else {
                if (!shooting & !gamepad2.yWasPressed()) {
                    intake.setAllPower(0); }
                if (ballCount == 0) {
                    signal.setPosition(1);
                }
            }


// DRIVE
            if(gamepad1.left_trigger>.1) {
                drive.parkMode();
            }

            if(gamepad1.left_trigger<.1) {
                drive.speedMode();
            }
// TURRET
            if(gamepad2.xWasPressed()) {
                turretOverride = !turretOverride;
                if(turretOverride) {
                    Turret.tracking = false;
                    turret.setPosition(0);
                }
            }

            if (gamepad1.y && gamepad1.dpad_left) {
                turret.resetRobotPose(resetPose);
            }
// TILT
            if (gamepad1.xWasPressed()) {
                tilting = true;
            }
            if (gamepad1.xWasReleased()) {
                tilting = false;
            }
            if (tilting) {
                tilt.tiltSetPower(1);
            }
            if (!tilting) {
                tilt.tiltSetPower(0);
            }

            /*HOOD
            hood.calcHoodPos(Math.sqrt(
                    (turret.distanceToBasket().x * turret.distanceToBasket().x) + (turret.distanceToBasket().y * turret.distanceToBasket().y)));
            */


            // update all systems
            /*
            drive.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            */



            //follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            drive.update(-gamepad1.left_stick_x, -gamepad1.left_stick_y , -gamepad1.right_stick_x);
            intake.update();
            turret.update();
            shooter.update();
            hood.update();
            gate.update();
            //signal.update();
            rail.update();
            bottomSensor.update();
            middleSensor.update();
            topSensor.update();
            follower.update();

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
