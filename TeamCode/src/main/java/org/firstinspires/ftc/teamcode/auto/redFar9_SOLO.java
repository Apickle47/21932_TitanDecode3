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

@Autonomous(name = "Red Solo Far 9")
public class redFar9_SOLO extends LinearOpMode {

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
        SHOOT_PRE, SPIKE3, RETURN_SHOOT3, SHOOT3, HUMAN, RETURN_SHOOT_H, SHOOT_H, LEAVE
    }

    PathState pathState;


    //1 degree = 0.01745329251994329576923690768489 rad
    //1 inch = 1 inch

    private final Pose startPose = new Pose(57.9258, 9.222, Math.PI).mirror();
    private final Pose shootPose = new Pose(58.9258, 14.222, Math.PI).mirror();
    private final Pose setUp3 = new Pose(92,28, Math.toRadians(0));
    private final Pose spike3 = new Pose(132, 38, Math.toRadians(0));
    private final Pose humanSetup = new Pose(13.79, 29.5, -1.9325).mirror();
    private final Pose human = new Pose(11.042, 12.7476, -1.7325).mirror();
    private final Pose leavePose = new Pose(50.9413, 35.391, Math.PI).mirror();

    private PathChain spikeThree, returnToShoot3, humanIntake, returnToShootH, leave;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(startPose, setUp3))
                .setLinearHeadingInterpolation(startPose.getHeading(), setUp3.getHeading())
                .addPath(new BezierLine(setUp3, spike3))
                .setLinearHeadingInterpolation(setUp3.getHeading(), spike3.getHeading())
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, shootPose))
                .setLinearHeadingInterpolation(spike3.getHeading(), shootPose.getHeading())
                .build();
        humanIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanSetup))
                .setLinearHeadingInterpolation(shootPose.getHeading(), humanSetup.getHeading())
                .addPath(new BezierLine(humanSetup, human))
                .setLinearHeadingInterpolation(humanSetup.getHeading(), human.getHeading())
                .build();
        returnToShootH = follower.pathBuilder()
                .addPath(new BezierLine(human, shootPose))
                .setLinearHeadingInterpolation(human.getHeading(), shootPose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
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
                intake.setRollerPower(0.25);
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

        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        util = new Util();
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, startPose, follower);
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, startPose);
        signal = new Signal(hardwareMap, util.deviceConf);
        rail = new Rail(hardwareMap, util.deviceConf);
        bottomSensor = new BottomSensor(hardwareMap, util.deviceConf);
        middleSensor = new MiddleSensor(hardwareMap, util.deviceConf);
        topSensor = new TopSensor(hardwareMap, util.deviceConf);
        double offset = -20;
        double hoodO = 0;

        turret.setBasketPos(Turret.redBasket);
        follower.setPose(startPose);
        buildPaths();

        StateMachine machine = new StateMachineBuilder()
                .waitState(2)
                .onEnter( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(hoodO, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                })
                .state(PathState.SHOOT_PRE)
                .onEnter( () -> {
                    intaking=false;
                    shooting=true;
                    gate.setPosition(Gate.OPEN);
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(hoodO, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                })
                .loop( () -> {
                    gate.setPosition(Gate.OPEN);
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(hoodO, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    if (shooter.safeToShoot((int)shooterTargetVel)) {
                        intake.setAllPower(1);
                    }
                })
                .onExit( () -> {
                    gate.setPosition(Gate.CLOSE);
                })
                .transitionTimed(2, PathState.SPIKE3)




                .state(PathState.SPIKE3)
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
                .transition( () ->  follower.atPose(spike3, 0.5, 0.5, Math.toRadians(5)), PathState.RETURN_SHOOT3)
                .transitionTimed(2, PathState.RETURN_SHOOT3)


                .state(PathState.RETURN_SHOOT3)
                .onEnter( () -> {
                    follower.followPath(returnToShoot3, true);
                })
                .loop( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(hoodO, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    intaking = false;
                    shooting = false;
                })
                .transition( () -> follower.atPose(shootPose, 0.25, 0.25), PathState.SHOOT3)


                .state(PathState.SHOOT3)
                .onEnter( () -> {
                    shooting = true;
                })
                .loop( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(hoodO, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = true;
                    gate.setPosition(Gate.OPEN);
                    intaking = false;
                    intake.setAllPower(1);
                })
                .transition( () -> (ballCount == 0), PathState.HUMAN)
                .transitionTimed(2, PathState.HUMAN)





                .state(PathState.HUMAN)
                .onEnter( () -> {
                    shooting = false;
                    intaking = true;
                    follower.followPath(humanIntake, true);
                })
                .loop( () -> {
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    shooting = false;
                    intaking = true;
                })
                .transition( () ->  (ballCount == 3 || follower.atPose(human, 0.5, 0.5)), PathState.RETURN_SHOOT_H)
                .transitionTimed(4, PathState.RETURN_SHOOT_H)

                .state(PathState.RETURN_SHOOT_H)
                .onEnter( () -> {
                    follower.followPath(returnToShootH, true);
                })
                .loop( () -> {
                    double goalDist = Math.sqrt(Math.pow(turret.distanceToBasket().getX(), 2) + Math.pow(turret.distanceToBasket().getY(), 2));
                    hood.hoodIncrement(hoodO, ShooterTable.getShotSolution(goalDist).getHoodP());
                    shooterTargetVel = (int)ShooterTable.getShotSolution(goalDist).getRpm() + offset;
                    shooter.setVelocity(shooterTargetVel);
                    ballCount = bottomSensor.hasBall() + middleSensor.hasBall() + topSensor.hasBall();
                    intaking = false;
                    shooting = false;
                })
                .transition( () -> follower.atPose(shootPose, 0.5, 0.5, Math.toRadians(5)), PathState.SHOOT_H)
                .transitionTimed(3, PathState.SHOOT_H)

                .state(PathState.SHOOT_H)
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
                .transition( () -> (ballCount == 0), PathState.LEAVE)
                .transitionTimed(2, PathState.LEAVE)

                .state(PathState.LEAVE)
                .onEnter( () -> {
                    gate.setPosition(Gate.CLOSE);
                    shooting = false;
                    follower.followPath(leave, true);
                    shooter.setVelocity(0);
                })
                .loop( () -> {
                })
                .onExit( () -> {
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
