package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@Autonomous(name = "RedGoalPreloadAuto", group = "Auto")
public class RedGoalPreloadAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 1500, intakeDuration = 1500;
    private long timer = System.currentTimeMillis();
    private long t = System.currentTimeMillis();
    private boolean firstShot = true;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-71 + ROBOT_BACK_LENGTH, 24.25 + ROBOT_WIDTH / 2, 0));

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

        //robot.shooter.setShooter(Shooter.Dist.CLOSE);

        t = System.currentTimeMillis();
        robot.shooter.reqAim(true);
        shoot(Math.PI / 4, true);
        intake(12, 60);
        open_gate();
        shoot(Math.PI, true);
        intake(-12, 54);
        shoot(Math.PI, true);
        intake(35, 60);
        shoot(Math.PI, false);
        //robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(0, 45, Math.PI / 2), 1.0);

        long x = System.currentTimeMillis() - t;

        TelemetryUtil.packet.put("Time : ", x);



        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading, boolean recharge) {
        //robot.drivetrain.goToPoint(new Pose2d(-12, 24, heading), 1.0, true);
        //robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.drivetrain.goToPoint(new Pose2d(-18, 18, heading), 0.6);
        robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
        robot.waitFor(300);
        if(firstShot){
            robot.waitFor(300);
            firstShot = false;
        }

        robot.shooter.reqShoot(true);
        robot.update();
        robot.waitFor(shootDuration);

        robot.shooter.reqStop(true);
        if (recharge) robot.shooter.reqAim(true);
    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 0.5);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 0.4);
        robot.waitFor(intakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(x, 48, Math.PI / 2), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqOff(true);
    }

    private void open_gate() {
        robot.drivetrain.goToPoint(new Pose2d(0, 48, Math.PI / 2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(0, 53, Math.PI / 2), 0.5);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 1000);

        robot.drivetrain.goToPoint(new Pose2d(0, 30, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }
}
