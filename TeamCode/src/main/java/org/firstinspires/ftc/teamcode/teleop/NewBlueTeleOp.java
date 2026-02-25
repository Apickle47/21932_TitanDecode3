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
@TeleOp(name = "NewBlueTeleOp")
public class NewBlueTeleOp extends OpMode {

    public static Pose resetPose = new Pose(72,72,Math.toRadians(270) );
    Util util = new Util();

    Intake intake;
    Turret turret;
    Mortar shooter;
    BottomSensor bottomSensor;
    MiddleSensor middleSensor;
    TopSensor topSensor;
    Gate gate;
    Hood hood;
    Rail rail;
    Signal signal;
    Tilt tilt;
    Follower follower;
    Pose pose;
    ElapsedTime myStopwatch = new ElapsedTime();
    TelemetryManager telemetryM;
    private double time;


    //


    int arState = 0;
    int abState = 0;
    double driveSpeed = 1.0;
    boolean turretOverride = false;
    private int servoPower;
    int[] powers = {-1, 0, 1};
    int count = 1;
    int incCount = 0;
    int topColor = 0;
    double[] intakeLightSequence = {0.28, 0.32, 0.36, 0.62};
    double[] StorageLightSequence = {1.0, 0.71, 0.5};
    double goalDist = Math.sqrt(Math.pow(72, 2) + Math.pow(72, 2));

    // Add more light sequences here if needed

    double[][] lightSequences = {intakeLightSequence, StorageLightSequence};
    int actSeq = 1;
    int shooterTargetVel = 0;
    int ballCount = 0;


    @Override
    public void init() {
        intake = new Intake(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(72, 72, (3 * Math.PI) / 2));
        shooter = new Mortar(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(72, 72, (3 * Math.PI) / 2));
        rail = new Rail(hardwareMap, util.deviceConf);
        signal = new Signal(hardwareMap, util.deviceConf);
        tilt = new Tilt(hardwareMap, util.deviceConf);
        follower = Constants.createFollower(hardwareMap);
        Pose pose;
        ElapsedTime myStopwatch = new ElapsedTime();
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));
        //sensor.setLEDBrightness(brightness);
        follower.startTeleOpDrive(true);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    public void start() {
        turret.setBasketPos(Turret.blueBasket);
        follower.update();
        hood.setHoodPosition(0.6);
        rail.setPosition(Rail.INDEX);
        Turret.tracking = false;

    }

    public void loop() {
        //Constant update variables
        follower.setTeleOpDrive(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, -gamepad1.right_stick_x * driveSpeed);
        ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
        pose = turret.getPose();
        topColor = topSensor.hasBall();
        goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
        hood.hoodIncrement(0.05 * incCount, goalDist > 115 ? Hood.farHood : Hood.closeHood);
        shooterTargetVel = shooter.calcVelocity(goalDist);


        //  ---  CONTROLS  ---

        // Shooting On/Off
        if (gamepad1.rightBumperWasPressed()) {
            if (arState == 1) {
                arState = 0;
            }
            else {
                arState = 1;
                abState = 0;
            }
        }
        // Intake
        if (gamepad2.rightBumperWasPressed()) { abState = 1; }
        if (gamepad2.leftBumperWasPressed()) { abState = 0; }
        // Tilt
        if (gamepad1.xWasPressed()) {
            count += 1;
            if (count > 2) {
                count = 0;
            }
            servoPower = powers[count];
            arState = 2;
        }
        // Reset Position
        if (gamepad1.y && gamepad1.leftBumperWasPressed()) {
            turret.resetRobotPose(resetPose);
        }
        // Slow Mode
        if (gamepad1.left_trigger > 0.1) {
            driveSpeed = 0.3;
        }
        else {
            driveSpeed = 1;
        }
        // Intake Eject
        if (gamepad2.yWasPressed()) { abState = 2; }
        if (gamepad2.yWasReleased()) { abState = 0; }
        // Preshoot
        if (gamepad2.bWasPressed()) { if (arState != 1) { arState = 3; } }
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
        // Turret Manual Override
        if (gamepad2.xWasPressed()) {
            turretOverride = !turretOverride;
        }
        // Turret Angle Offset
        if (gamepad2.dpadLeftWasPressed()) {
            turret.setAngleOffset(2);
        }
        if (gamepad2.dpadRightWasPressed()) {
            turret.setAngleOffset(-2);
        }
        // Indexer
        if (gamepad2.aWasPressed()) {
            if(rail.getPosition() == Rail.INDEX) {
                rail.setPosition(Rail.INLINE);
            }
            else {
                rail.setPosition(Rail.INDEX);
            }
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
                if (!turretOverride) { Turret.tracking = true; }
                if (shooter.safeToShoot(shooterTargetVel)) {
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);
                    abState = 1;
                }

                break;

            case(2):
                actSeq = 2;
                myStopwatch.reset();
                shooter.setVelocity(0);
                abState = 3;
                tilt.tiltSetPower(servoPower);
                break;
            case(3):
                if (!turretOverride) { Turret.tracking = true; }
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
        double liftTime = 6;
        switch(actSeq) {
            case(0):
                signal.setPosition(lightSequences[0][ballCount]);
                break;
            case(1):
                switch(topColor) {
                    case(0):
                        signal.setPosition(1);
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
                signal.setPosition(myStopwatch.seconds() >= 0.5 && signal.getLEDColor() == 0.62 && time < liftTime ? 0.28 : 0.62);
                if (myStopwatch.time() >= 0.5 || time >= liftTime) {
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
        tilt.update();


        //Telemetry
        telemetryM.addData("Arush State", arState);
        telemetryM.addData("Abhay State", abState);
        telemetryM.addLine("");
        telemetryM.addLine("SHOOTER:");
        telemetryM.addData("Shooter vel", shooter.getVelocity());
        telemetryM.addData("Shooter target vel", shooter.getTargetVelocity());
        //  telemetryM.addData("Keep Shooter Running", keepShooterRunning);
        //   telemetryM.addData("Preshoot", preshoot);
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
        telemetryM.addData("Turret Angle Offset", Turret.angleOffset);
        telemetryM.addLine("");
        telemetryM.addLine("MISC:");
        telemetryM.addData("Ball Count", ballCount);
        telemetryM.addData("hood angle", hood.getHoodPosition());
        telemetryM.addData("LED Color", signal.getLEDColor());
        telemetryM.addData("Rail Position", rail.getPosition());
        telemetryM.addData("Tilt Power", servoPower);
        telemetryM.addData("Tilt Actual Power", tilt.tiltGetPower());

        //    telemetryM.addData("Intaking?", intaking);
        telemetryM.addLine("");
        telemetryM.addLine("COLOR SENSOR");
        telemetryM.addData("Bottom Sensor Color", bottomSensor.getColor());
        telemetryM.addData("Middle Sensor Color", middleSensor.getColor());
        telemetryM.addData("Top Sensor Color", topSensor.getColor());


        telemetryM.update(telemetry);
        telemetryM.update();

    }
}


