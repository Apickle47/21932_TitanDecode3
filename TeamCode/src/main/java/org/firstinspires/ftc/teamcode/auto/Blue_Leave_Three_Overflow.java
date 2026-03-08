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
import org.firstinspires.ftc.teamcode.subsystems.Signal;
import org.firstinspires.ftc.teamcode.subsystems.TopSensor;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Util;

import java.util.List;

@Autonomous
public class Blue_Leave_Three_Overflow extends LinearOpMode {

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

    public static int KICKER_WAIT_TIME = 600;

    private int shooterTargetSpeed;
    private int launchCount, shootPoseCount, launchIf;
    private double target;
    private PathState[] idleShoot;
    private Pose[] idleShootPose;
    private int count;
    private int gintakeCount;
    ElapsedTime time1 = new ElapsedTime();


    public enum PathState {
        DRIVE_START_POS_SHOOT_POS, SHOOT_PRE, SPIKE_ONE, RETURN_SHOOT1, SHOOT_ONE, SET_UP2, SPIKE_TWO, RETURN_SHOOT2, SHOOT_TWO, SET_UP3, SPIKE_THREE, RETURN_SHOOT3, SET_UP_HUMAN, HUMAN, RETURN_SHOOT_HUMAN, GINTAKE_AWAY, GINTAKE, RETURN_SHOOT_GINTAKE, GINTAKE_SHOOT, GINTAKE_AWAY2, GINTAKE2, RETURN_SHOOT_GINTAKE2, GINTAKE_SHOOT2, GINTAKE_SETUP, DONE, IDLE_SHOOT, IDLE_GATE, IDLE_GATE2, HIT_GATE1, HIT_GATE2, SHOOT_THREE
    }

    PathState pathState;

    private final Pose startPose = new Pose(56.33,8.683, Math.toRadians(90));
    private final Pose scanShootPose = new Pose(85.8,75.24, Math.toRadians(90));
    private final Pose PreshootPose = new Pose(35.511, 8.77, Math.toRadians(38.937)).mirror();
    private final Pose shootPose = new Pose(85.8,75.24, 0).mirror();

    private final Pose lastShootPose = new Pose(84,109, 0).mirror();
    private final Pose spike1 = new Pose(132.7, 84.14, Math.toRadians(0)).mirror();
    private final Pose setUp2 = new Pose(95, 58.2, Math.toRadians(0)).mirror();
    private final Pose spike2 = new Pose(135.5,58.2,Math.toRadians(0)).mirror();
    private final Pose setUp3 = new Pose(95,35, Math.toRadians(0)).mirror();
    private final Pose spike3 = new Pose(140, 35, Math.toRadians(0)).mirror();
    private final Pose setUpH = new Pose(128, 52, Math.toRadians(-90)).mirror();
    private final Pose humanPose = new Pose(130,7, Math.toRadians(-90)).mirror();
    private final Pose gintakeAwayPose1 = new Pose(102.65, 60.4, 0).mirror();
    private final Pose gintakePose = new Pose(140.51, 59.58, 0.4860).mirror();  //1 degree = 0.01745329251994329576923690768489 rad
    private final Pose hitGate = new Pose(132.105, 70, Math.toRadians(-90)).mirror();
    private final Pose hitGateRev = new Pose(132,60.6, Math.toRadians(-90)).mirror();


    private PathChain driveStartPosShootPos, spikeOne, spikeTwo, spikeThree, returnToShoot1, returnToShoot2, returnToShoot3, setUpTwo, setUpThree, setUpHuman, human, returnShootHuman, gintakeAway, gintake, returnToShootGintake, gintakeSetup;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PreshootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addParametricCallback(.0, () -> follower.setMaxPower(1))
                .addParametricCallback(.5, () -> intake.setAllPower(1))
//                .addParametricCallback(.85, () -> follower.setMaxPower(0.3))
//                .addParametricCallback(0.99, () -> follower.setMaxPower(0.8))
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
        idleShoot = new PathState[]{PathState.SPIKE_ONE, PathState.SPIKE_TWO, PathState.GINTAKE};
        count = 0;
        double shootTime = 0.4;
        gintakeCount = 1;
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(117.771,126.922,Math.toRadians(38.937)), follower);
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(117.771,126.922,Math.toRadians(38.937)));
        signal = new Signal(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);

        turret.setBasketPos(Turret.redBasket);
        follower.setPose(startPose);
        buildPaths();

        StateMachine machine = new StateMachineBuilder()
                //good
                .state(PathState.DRIVE_START_POS_SHOOT_POS)
                .onEnter( () -> {
                    hood.setHoodPosition(0.55);
                    //turret.setAngleOffset(4);
                    follower.followPath(driveStartPosShootPos, true);
                    shooting=true;
                })
                .loop( () -> {
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                })
                //.transition( () -> follower.atPose(PreshootPose, 1, 1, Math.toRadians(6)), PathState.SET_UP2)




                .build();



        waitForStart();
        //shooter.setVelocity(1485);
        machine.start();

        opModeTimer.resetTimer();
        Turret.tracking = true;
        hood.setHoodPosition(.60);
        rail.setPosition(Rail.INDEX);
        //shooter.setVelocity(shooter.calcVelocity((71-20)*Math.sqrt(2)));
        //shooter.setVelocity(1485);
        ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();

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
            //telemetry.addData("count", count);
            telemetry.addData("ball count", ballCount);
            telemetry.addData("gintakeCount", gintakeCount);

            telemetry.update();
        }
    }
}
