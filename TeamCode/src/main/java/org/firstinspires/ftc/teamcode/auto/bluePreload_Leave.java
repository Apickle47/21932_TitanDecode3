package org.firstinspires.ftc.teamcode.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BottomSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MiddleSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mortar;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTable;
import org.firstinspires.ftc.teamcode.subsystems.Signal;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Util;

import java.util.List;

@Autonomous(name = "Blue Preload + Leave")
public class bluePreload_Leave extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int ballCount;
    private boolean shooting, intaking = false;

    Util util;
    Mortar shooter;
    Turret turret;
    Intake intake;
    Gate gate;
    Hood hood;
    Signal signal;
    Rail rail;
    BottomSensor bottomSensor;
    MiddleSensor middleSensor;
    TopSensor topSensor;


    private double shooterTargetVel;



    public enum PathState {
        DRIVE_START_POS_SHOOT_POS
    }

    PathState pathState;


    //1 degree = 0.01745329251994329576923690768489 rad
    //1 inch = 1 inch

    private final Pose startPose = new Pose(26.678,126.578, 2.4534);
    private final Pose lastShootPose = new Pose(84,109, 0).mirror();

    private PathChain driveStartPosShootPos;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lastShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), lastShootPose.getHeading())
                .addParametricCallback(.4, () -> follower.setMaxPower(0.8))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
    }

    private void intakeBalls() {
        if(intaking) {
            if (ballCount == 0) {
                signal.setPosition(Signal.VIOLET);
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(1);

            } else if (ballCount == 1) {
                signal.setPosition(Signal.RED);
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(1);

            } else if (ballCount == 2) {
                signal.setPosition(Signal.YELLOW);
                gate.setPosition(Gate.CLOSE);
                intake.setRollerPower(0);
                intake.setIntakePower(1);

            } else if (ballCount == 3) {
                signal.setPosition(Signal.GREEN);
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(0);
            }
        } else if (!shooting) {
            intake.setAllPower(0);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(26.678,126.578, 2.4534), follower);
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(26.678,126.578, 2.4534));
        signal = new Signal(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        double offset = 0;

        turret.setBasketPos(Turret.blueBasket);
        follower.setPose(startPose);
        buildPaths();

        StateMachine machine = new StateMachineBuilder()
                //good
                .state(PathState.DRIVE_START_POS_SHOOT_POS)
                .onEnter( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    follower.followPath(driveStartPosShootPos, true);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                })
                .build();



        waitForStart();

        opModeTimer.resetTimer();
        Turret.tracking = true;
        rail.setPosition(Rail.INDEX);
        ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();

        machine.start();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while(opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            follower.update();
            machine.update();
            intakeBalls();

            ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
            shooter.update();
            turret.update();
            intake.update();
            gate.update();
            hood.update();
            signal.update();
            rail.update();
            bottomSensor.update();
            middleSensor.update();
            topSensor.update();

//TELEMETRY
            telemetry.addData("path state", pathState.toString());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("opModeTimer", opModeTimer.getElapsedTimeSeconds());
            telemetry.addData("ball count", ballCount);
            telemetry.addData("shooter vel", shooter.getVelocity());

            telemetry.update();
        }
    }
}
