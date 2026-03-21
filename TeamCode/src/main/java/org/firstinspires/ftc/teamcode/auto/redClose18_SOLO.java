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

@Autonomous(name = "Red Solo 18 Close")
public class redClose18_SOLO extends LinearOpMode {

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


    private int gintakeCount;
    private double shooterTargetVel;



    public enum PathState {
        DRIVE_START_POS_SHOOT_POS, SPIKE_ONE, RETURN_SHOOT1, SHOOT_ONE, SPIKE_TWO, RETURN_SHOOT2, SHOOT_TWO, SPIKE_THREE, RETURN_SHOOT3, GINTAKE_AWAY, GINTAKE, RETURN_SHOOT_GINTAKE, GINTAKE_SHOOT, GINTAKE_AWAY2, GINTAKE2, RETURN_SHOOT_GINTAKE2, GINTAKE_SHOOT2, IDLE_GATE, IDLE_GATE2, SHOOT_THREE
    }

    PathState pathState;


    //1 degree = 0.01745329251994329576923690768489 rad
    //1 inch = 1 inch

    private final Pose startPose = new Pose(26.678,126.578, 2.4534).mirror();
    private final Pose PreshootPose = new Pose(85.8,75.24, Math.toRadians(38.937));
    private final Pose shootPose = new Pose(85.8,79.24, 0);
    private final Pose lastShootPose = new Pose(84,109, 0);
    private final Pose spike1 = new Pose(131, 84.14, Math.toRadians(0));
    private final Pose setUp2 = new Pose(95, 58.2, Math.toRadians(0));
    private final Pose leave2 = new Pose(110, 58.2, Math.toRadians(0));
    private final Pose spike2 = new Pose(133,59.2,Math.toRadians(0));
    private final Pose setUp3 = new Pose(93,35, Math.toRadians(0));
    private final Pose spike3 = new Pose(135, 35, Math.toRadians(0));
    private final Pose gintakeAwayPose1 = new Pose(102.65, 57.4, 0);
    private final Pose gintakePose = new Pose(12.47, 58.06086, 2.568).mirror();

    private PathChain driveStartPosShootPos, spikeOne, spikeTwo, spikeThree, returnToShoot1, returnToShoot2, returnToShoot3, gintakeAway, gintake, returnToShootGintake;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PreshootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addParametricCallback(.4, () -> follower.setMaxPower(0.8))
                .addParametricCallback(.5, () -> intake.setAllPower(1))
                .addParametricCallback(0.99, () -> follower.setMaxPower(1))
                .build();
        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1))
                .setConstantHeadingInterpolation(spike1.getHeading())
                .build();

        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1, lastShootPose))
                .setLinearHeadingInterpolation(spike1.getHeading(), shootPose.getHeading())
                .build();
        spikeTwo = follower.pathBuilder()
                .addPath(new BezierLine(PreshootPose, setUp2))
                .setLinearHeadingInterpolation(startPose.getHeading(), setUp2.getHeading())
                .addPath(new BezierLine(setUp2,spike2))
                .setLinearHeadingInterpolation(setUp2.getHeading(), spike2.getHeading())
                .build();
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2, leave2))
                .setLinearHeadingInterpolation(spike2.getHeading(), leave2.getHeading())
                .addPath(new BezierLine(leave2, shootPose))
                .setLinearHeadingInterpolation(leave2.getHeading(), shootPose.getHeading())
                .build();
        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp3.getHeading())
                .addPath(new BezierLine(setUp3, spike3))
                .setLinearHeadingInterpolation(setUp3.getHeading(), spike3.getHeading())
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, shootPose))
                .setLinearHeadingInterpolation(spike3.getHeading(), shootPose.getHeading())
                .build();
        gintake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gintakeAwayPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gintakeAwayPose1.getHeading())
                .addPath(new BezierLine(gintakeAwayPose1, gintakePose))
                .setLinearHeadingInterpolation(gintakeAwayPose1.getHeading(), gintakePose.getHeading())
                .build();
        returnToShootGintake = follower.pathBuilder()
                .addPath(new BezierLine(gintakePose, gintakeAwayPose1))
                .setLinearHeadingInterpolation(gintakePose.getHeading(), gintakeAwayPose1.getHeading())
                .addPath(new BezierLine(gintakeAwayPose1, shootPose))
                .setLinearHeadingInterpolation(gintakeAwayPose1.getHeading(), shootPose.getHeading())
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
        double shootTime = 0.4;
        gintakeCount = 1;
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(26.678,126.578, 2.4534).mirror(), follower);
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(26.678,126.578, 2.4534).mirror());
        signal = new Signal(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        double offset = 0;

        turret.setBasketPos(Turret.redBasket);
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
                    shooting=true;
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    gate.setPosition(Gate.OPEN);
                })
                .transition( () -> follower.atPose(PreshootPose, 1, 1, Math.toRadians(6)), PathState.SPIKE_TWO)




                .state(PathState.SPIKE_TWO)
                .onEnter( () -> {
                    gate.setPosition(Gate.CLOSE);
                    shooter.setVelocity(shooterTargetVel);
                    shooting = false;
                    follower.followPath(spikeTwo, true);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    gate.setPosition(Gate.CLOSE);
                    shooting = false;
                    intaking = true;
                })
                .transition( () -> ballCount == 3 || follower.atPose(spike2, 0.5, 0.5), PathState.RETURN_SHOOT2)
                .transitionTimed(2, PathState.RETURN_SHOOT2)

                .state(PathState.RETURN_SHOOT2)
                .onEnter( () -> {
                    gate.setPosition(Gate.OPEN);
                    follower.followPath(returnToShoot2, true);
                    intake.setAllPower(0);
                    intaking = true;

                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    gate.setPosition(Gate.OPEN);
                })
                .onExit( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    gate.setPosition(Gate.CLOSE);
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5), PathState.SHOOT_TWO)

                .state(PathState.SHOOT_TWO)
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    gate.setPosition(Gate.OPEN);
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    shooting = true;
                    intaking = false;
                    intake.setAllPower(1);
                })
                .transitionTimed(1, PathState.GINTAKE)





                .state(PathState.GINTAKE)
                .onEnter( () -> {
                    shooting = false;
                    gate.setPosition(Gate.CLOSE);
                    follower.followPath(gintake, true);
                })
                .loop( () -> {
                    shooting = false;
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    gate.setPosition(Gate.CLOSE);
                    intaking = true;
                })
                .transitionTimed(1.5, PathState.IDLE_GATE)

                .state(PathState.IDLE_GATE)
                .onEnter( () -> {
                    gate.setPosition(Gate.CLOSE);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    intaking = true;
                })
                .transition( () -> ballCount == 3, PathState.RETURN_SHOOT_GINTAKE)
                .transitionTimed(3, PathState.RETURN_SHOOT_GINTAKE)

                .state(PathState.RETURN_SHOOT_GINTAKE)
                .onEnter( () -> {
                    shooting = true;
                    follower.followPath(returnToShootGintake, true);
                    intake.setAllPower(0);
                    intaking = true;
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    intaking = false;
                    gate.setPosition(Gate.OPEN);
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5, Math.toRadians(5)), PathState.GINTAKE_SHOOT)

                .state(PathState.GINTAKE_SHOOT)
                .onEnter( () -> {
                    //turret.setAngleOffset(-4);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intaking = false;
                    intake.setAllPower(1);
                })
                .transitionTimed(shootTime, PathState.GINTAKE2)





                .state(PathState.GINTAKE2)
                .onEnter( () -> {
                    shooting = false;
                    gate.setPosition(Gate.CLOSE);
                    follower.followPath(gintake, true);
                })
                .loop( () -> {
                    shooting = false;
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    gate.setPosition(Gate.CLOSE);
                    intaking = true;
                })
                .transitionTimed(1, PathState.IDLE_GATE2)

                .state(PathState.IDLE_GATE2)
                .onEnter( () -> {
                    gate.setPosition(Gate.CLOSE);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    intaking = true;

                })
                .transition( () -> ballCount == 3, PathState.RETURN_SHOOT_GINTAKE2)
                .transitionTimed(3, PathState.RETURN_SHOOT_GINTAKE2)

                .state(PathState.RETURN_SHOOT_GINTAKE2)
                .onEnter( () -> {
                    shooting = true;
                    follower.followPath(returnToShootGintake, true);
                    intake.setAllPower(0);
                    intaking = true;
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    intaking = false;
                    gate.setPosition(Gate.OPEN);
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5, Math.toRadians(5)), PathState.GINTAKE_SHOOT2)

                .state(PathState.GINTAKE_SHOOT2)
                .onEnter( () -> {
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intaking = false;
                    intake.setAllPower(1);

                })
                .transitionTimed(shootTime, PathState.SPIKE_THREE)





                .state(PathState.SPIKE_THREE)
                .onEnter( () -> {
                    shooting = false;
                    intaking = true;
                    follower.followPath(spikeThree, true);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = false;
                    intaking = true;
                })
                .transition( () ->  ballCount == 3 || follower.atPose(spike3, 0.5, 0.5, Math.toRadians(5)), PathState.RETURN_SHOOT3)
                .transitionTimed(4, PathState.RETURN_SHOOT3)


                .state(PathState.RETURN_SHOOT3)
                .onEnter( () -> {
                    follower.followPath(returnToShoot3, true);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    intaking = false;
                    shooting = false;
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5), PathState.SHOOT_THREE)


                .state(PathState.SHOOT_THREE)
                .onEnter( () -> {
                    shooting = true;
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intaking = false;
                    intake.setAllPower(1);
                })
                .transitionTimed(shootTime, PathState.SPIKE_ONE)






                .state(PathState.SPIKE_ONE)
                .onEnter( () -> {
                    shooting = false;
                    follower.followPath(spikeOne, true);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    intaking = true;
                })
                .transitionTimed(2, PathState.RETURN_SHOOT1)
                .transition( () -> ballCount == 3 || follower.atPose(spike1, 0.5, 0.5), PathState.RETURN_SHOOT1)

                .state(PathState.RETURN_SHOOT1)
                .onEnter( () -> {
                    //turret.setAngleOffset(-2);
                    follower.followPath(returnToShoot1, true);
                    intaking = false;
                })
                .loop( () -> {
                    gate.setPosition(Gate.OPEN);
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(0, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                })
                .transition( () -> follower.atPose(lastShootPose, .5, .5), PathState.SHOOT_ONE)

                .state(PathState.SHOOT_ONE)
                .loop( () -> {
                    shooting = true;
                    intaking = false;
                    intake.setAllPower(1);
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
            telemetry.addData("gintakeCount", gintakeCount);
            telemetry.addData("shooter vel", shooter.getVelocity());

            telemetry.update();
        }
    }
}
