package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "RedGoal18Auto", group = "Auto")
public class RedGoal18Ball extends LinearOpMode {
    private Robot robot;
    private double timer = System.currentTimeMillis();
    long delay;
    private final long shootDuration = 900, intakeDuration = 1000;
    private boolean firstShot = true;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-72 + ROBOT_BACK_LENGTH, 24 + ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.IDLE;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 250);
        }
        robot.sensors.light0G.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        //robot.drivetrain.goToPoint(new Pose2d(-40, 40, 0), 1.0);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.shooter.reqAim(true);
        shoot(Math.PI / 4, true);
        intake(10, 57);
        shoot(Math.PI, true);
        gate_intake();
        //shoot(Math.PI, true);
        //gate_intake();

        shoot(Math.PI, true);
        intake(-14, 50);
        shoot(Math.PI, true);
        intake(32, 57);
        shoot(Math.PI, false);

        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(-4, 43, Math.PI/2), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading, boolean recharge) {
        //robot.drivetrain.goToPoint(new Pose2d(-12, 24, heading), 1.0, true);
        //robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.drivetrain.goToPoint(new Pose2d(-18, 18, heading), 1.0);
        robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
        robot.waitFor(400);
        if(firstShot){
            robot.waitFor(300);
            firstShot = false;
        }

        robot.shooter.reqShoot(true);
        robot.update();
        robot.waitFor(shootDuration);

        robot.shooter.reqStop(true);

    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 0.7);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 0.7);
        robot.waitFor(intakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(x, 44, Math.PI / 2), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqOff(true);
        robot.shooter.reqAim(true);

    }

    private void open_gate() {
        robot.drivetrain.goToPoint(new Pose2d(0, 48, Math.PI/2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(0,53, Math.PI/2), 0.5);

        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 1000);

        robot.drivetrain.goToPoint(new Pose2d(0, 24, Math.PI/2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }

    private void gate_intake() {

        robot.drivetrain.goToPoint(new Pose2d(0, 35, Math.PI/2), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqIntake(true);

        //open gate
        timer = System.currentTimeMillis();

        robot.drivetrain.goToPoint(new Pose2d(5,57, Math.toRadians(118)), 0.8, true);
        robot.waitFor(200);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        //timer = System.currentTimeMillis();

        //go behind the gate to intake the balls

        //start farther from the gate
        robot.drivetrain.goToPoint(new Pose2d(21, 63, Math.toRadians(150)), 1.0);

        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.drivetrain.goToPoint(new Pose2d(13, 63, Math.toRadians(150)), 0.3);

        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || System.currentTimeMillis() - timer <= 2000);


        robot.drivetrain.goToPoint(new Pose2d(13, 36, Math.toRadians(150)), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);

        robot.shooter.reqAim(true);
    }
}
