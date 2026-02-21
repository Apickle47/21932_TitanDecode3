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
        double[][] lightSequence = {};
        double[] intakeLightSequence = {1.0, 0.29, 0.35, 0.62};
        double[] topBallLightSequence = {1.0, 0.71, 0.5};


        while(opModeIsActive()) {
            //Constant update variables
            ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
            pose = turret.getPose();
            shooterTargetVel = shooter.calcVelocity(Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2)));


            //  ---  CONTROLS  ---

            // Shooting
            if (gamepad1.rightBumperWasPressed()) {
                if (arState == 1) {
                    arState = 0;
                }
                else {
                    arState = 1;
                }
            }

            // Arush States
            switch (arState) {
                case(0):
                    if (abState != 0) {
                        intake.setAllPower(1);
                        //lightSequence = 1;
                    }

            }








        }
    }

}
