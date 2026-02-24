package org.firstinspires.ftc.teamcode.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous
public class AutoSyncClose extends OpMode{


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
    private double[] times;
    private int count;
    ElapsedTime time1 = new ElapsedTime();


    public enum PathState {
        DRIVE_START_POS_SHOOT_POS, SHOOT_PRELOAD, SPIKE_ONE, RETURN_SHOOT1, SET_UP2, SPIKE_TWO, RETURN_SHOOT2, SET_UP3, SPIKE_THREE, RETURN_SHOOT3, SET_UP_HUMAN, HUMAN, RETURN_SHOOT_HUMAN, GINTAKE_AWAY, GINTAKE, RETURN_SHOOT_GINTAKE, DONE, IDLE
    }

    PathState pathState;

    private final Pose startPose = new Pose(126.66,129.71, .6879);
    private final Pose shootPose = new Pose(88.7,83.7836, 0);
    private final Pose spike1 = new Pose(120, 85, Math.toRadians(0));
    private final Pose setUp2 = new Pose(96.25263157894737, 59.284210526315775, Math.toRadians(0));
    private final Pose spike2 = new Pose(126,59,Math.toRadians(0));
    private final Pose setUp3 = new Pose(96.25263157894737,35.368421052631575, Math.toRadians(0));
    private final Pose spike3 = new Pose(128, 35, Math.toRadians(0));
    private final Pose setUpH = new Pose(128, 52, Math.toRadians(270));
    private final Pose humanPose = new Pose(130,7, Math.toRadians(270));
    private final Pose gintakeAwayPose1 = new Pose(121.705, 58.694, .0454);
    private final Pose gintakePose = new Pose(137.432, 61.0279, 0.4873);
    private final Pose hitGate = new Pose(134.105, 82.3867, 1.511);
    private final Pose hitGateRev = new Pose(134.105, 82.3867, -1.511);


    private PathChain driveStartPosShootPos, spikeOne, spikeTwo, spikeThree, returnToShoot1, returnToShoot2, returnToShoot3, setUpTwo, setUpThree, setUpHuman, human, returnShootHuman, gintakeAway, gintake, returnToShootGintake;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration(5)
                .build();
        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike1.getHeading())
                .addPath(new BezierLine(spike1, hitGate))
                .setLinearHeadingInterpolation(spike2.getHeading(), hitGate.getHeading())
                .build();

        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(hitGate, shootPose))
                .setLinearHeadingInterpolation(hitGate.getHeading(), shootPose.getHeading())
                .build();
        setUpTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp2.getHeading())
                .build();
        spikeTwo = follower.pathBuilder()
                .addPath(new BezierLine(setUp2,spike2))
                .setLinearHeadingInterpolation(setUp2.getHeading(), spike2.getHeading())
                .addPath(new BezierLine(spike2, hitGateRev))
                .setLinearHeadingInterpolation(spike2.getHeading(), hitGateRev.getHeading())
                .build();
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(hitGateRev, shootPose))
                .setLinearHeadingInterpolation(hitGateRev.getHeading(), shootPose.getHeading())
                .build();
        setUpThree = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp3.getHeading())
                .build();
        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(setUp3, spike3))
                .setLinearHeadingInterpolation(setUp3.getHeading(), spike3.getHeading())
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, shootPose))
                .setLinearHeadingInterpolation(spike3.getHeading(), shootPose.getHeading())
                .build();
        setUpHuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUpH))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUpH.getHeading())
                .build();
        human = follower.pathBuilder()
                .addPath(new BezierLine(setUpH, humanPose))
                .setLinearHeadingInterpolation(setUpH.getHeading(), humanPose.getHeading())
                .build();
        returnShootHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, shootPose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), shootPose.getHeading())
                .build();
        gintake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gintakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gintakePose.getHeading())
                .build();
        gintakeAway = follower.pathBuilder()
                .addPath(new BezierLine(gintakePose, gintakeAwayPose1))
                .setLinearHeadingInterpolation(gintakePose.getHeading(), gintakeAwayPose1.getHeading())
                .build();

        returnToShootGintake = follower.pathBuilder()
                .addPath(new BezierLine(gintakeAwayPose1, shootPose))
                .setLinearHeadingInterpolation(gintakeAwayPose1.getHeading(), shootPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                // check is follower done its path?
                if (!follower.isBusy()) {
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                    gate.setPosition(Gate.CLOSE);
                    shooting = false;
                }
                if (!follower.isBusy() && !shooting) {
                    follower.followPath(spikeOne, true);
                    startIntake();
                    setPathState(PathState.SPIKE_ONE);
                }
                break;
            case SPIKE_ONE:
                if (!follower.isBusy()) {
                    intake.setAllPower(0);
                    follower.followPath(returnToShoot1, true);
                    setPathState(PathState.RETURN_SHOOT1);
                }
                break;
            case RETURN_SHOOT1:
                if (!follower.isBusy()) {
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);

                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {//TODO: TUNE THE TIME
                        gate.setPosition(Gate.CLOSE);
                        shooting = false;
                    }
                    if (!follower.isBusy() && !shooting) {
                        follower.followPath(setUpTwo, true);
                        setPathState(PathState.SET_UP2);
                    }
                }
                break;
            case SET_UP2:
                if (!follower.isBusy()) {
                    follower.followPath(spikeTwo, true);
                    startIntake();
                    setPathState(PathState.SPIKE_TWO);
                }
            case SPIKE_TWO:
                if (!follower.isBusy()) {
                    intake.setAllPower(0);
                    follower.followPath(returnToShoot2, true);
                    setPathState(PathState.RETURN_SHOOT2);
                }
            case RETURN_SHOOT2:
                if (!follower.isBusy()) {
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);

                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {//TODO: TUNE THE TIME
                        gate.setPosition(Gate.CLOSE);
                        shooting = false;
                    }
                    if (!follower.isBusy() && !shooting) {
                        follower.followPath(gintake, true);
                        startIntake();
                        setPathState(PathState.IDLE);
                    }
                }
                break;

//            case GINTAKE:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.IDLE);
//                }
            case IDLE:
                if (opModeTimer.getElapsedTimeSeconds() > times[count] && !follower.isBusy()){
                    follower.followPath(gintakeAway, true);
                    intake.setAllPower(0);
                    count++;
                    setPathState(PathState.GINTAKE_AWAY);
                }
            case GINTAKE_AWAY:
                gate.setPosition(Gate.CLOSE);
                if (!follower.isBusy() && opModeTimer.getElapsedTimeSeconds() > times[count]) {
                    follower.followPath(returnToShootGintake, true);
                    setPathState(PathState.RETURN_SHOOT_GINTAKE);
                }
                break;
            case RETURN_SHOOT_GINTAKE:
                gate.setPosition(Gate.CLOSE);
                if (!follower.isBusy()) {
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intake.setAllPower(1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.8) {//TODO: TUNE THE TIME
                    gate.setPosition(Gate.CLOSE);
                    shooting = false;
                }
                if (!follower.isBusy() && !shooting) {
                    follower.followPath(gintake, true);
                    startIntake();
                    setPathState(PathState.GINTAKE);
                }

                /*
            case SET_UP3:
                if (!follower.isBusy()) {
                    follower.followPath(spikeThree, true);
                    startIntake();
                    setPathState(PathState.SPIKE_THREE);
                }
            case SPIKE_THREE:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot3, true);
                    setPathState(PathState.RETURN_SHOOT3);
                }
            case RETURN_SHOOT3:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("four" + opModeTimer.getElapsedTime());
                    follower.followPath(setUpHuman, true);
                    setPathState(PathState.SET_UP_HUMAN);
                }
            case SET_UP_HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(human, true);
                    startIntake();
                    setPathState(PathState.HUMAN);
                }
            case HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(returnShootHuman);
                    setPathState(PathState.RETURN_SHOOT_HUMAN);
                }
            case RETURN_SHOOT_HUMAN:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("five" + opModeTimer.getElapsedTime());
                    setPathState(PathState.DONE);
                }
                */
            default:
                telemetry.addLine("Auto Finished" + opModeTimer.getElapsedTime());
                shooter.setFlyMotorPower(0);
                shooter.setFlyMotor2Power(0);
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    //INIT
    @Override
    public void init() {
        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(64.5, 16.4, Math.toRadians(42.5)));
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(64.5, 16.4, Math.toRadians(42.5)));
        signal = new Signal(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);

        times = new double[]{12.5, 18.5, 30};
        count = 0;

        turret.setBasketPos(Turret.redBasket);

        buildPaths();
        follower.setPose(startPose);
    }

    //START
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        Turret.tracking = true;
        hood.setHoodPosition(Hood.closeHood);
        rail.setPosition(Rail.INDEX);
        //shooter.setVelocity(shooter.calcVelocity((71-20)*Math.sqrt(2)));
        time1.reset();
        shooter.setVelocity(1400);
        ballCount = 3;
    }

    //LOOP
    @Override
    public void loop() {
//AUTONOMOUS
        follower.update();
        statePathUpdate();

//SUBSYSTEMS
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

        telemetry.update();
    }


    //    public void sleep(int t) {
//        try {
//            Thread.sleep(t); // Wait for 1 millisecond
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt(); // Restore interrupted status
//            // Optionally, log or handle the interruption
//        }
//    }
    public void Launch() {
        shooting = true;
        intake.setAllPower(1);
    }
    public void startIntake() {
        gate.setPosition(Gate.CLOSE);
        if (!shooting) {
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
        }
        else {
            gate.setPosition(Gate.CLOSE);
            if (!shooting) {
                gate.setPosition(Gate.CLOSE);
                intake.setAllPower(0);
            }
            if (ballCount == 0) {
                gate.setPosition(Gate.CLOSE);
                signal.setPosition(1);
            }
        }
    }
}
