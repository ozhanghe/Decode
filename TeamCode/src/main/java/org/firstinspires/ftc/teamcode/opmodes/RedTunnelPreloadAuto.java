package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Autonomous(name = "RedTunnelPreloadAuto", group = "Auto")
public class RedTunnelPreloadAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 1200;
    private long timer = System.currentTimeMillis();

    public static double heading = 2.71;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(72 - ROBOT_BACK_LENGTH, 24 - ROBOT_WIDTH / 2, Math.PI));

        robot.shooter.state = Shooter.State.IDLE;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 250);
        }
        robot.sensors.light0G.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        robot.shooter.reqAim(true);
        shoot(heading);
        intake(12, 58);

        open_gate();

        shoot(heading);
        intake(36, 58);
        shoot(heading);
        intake_far();
        shoot(heading);

        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(0, 45, Math.PI / 2), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading) {
        //robot.drivetrain.goToPoint(new Pose2d(-12, 24, heading), 1.0, true);
        //robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.drivetrain.goToPoint(new Pose2d(53, 12, heading), 1.0);
        robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
        robot.waitFor(200);

        robot.shooter.reqShoot(true);
        robot.update();
        robot.waitFor(shootDuration);

        robot.shooter.reqStop(true);

    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.toRadians(90)), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(90)), 0.4);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 3500);

        robot.drivetrain.goToPoint(new Pose2d(x, 44, Math.PI / 2), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqOff(true);
        robot.shooter.reqAim(true);
    }

    private void open_gate() {
        robot.drivetrain.goToPoint(new Pose2d(0, 48, Math.PI / 2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(0, 53, Math.PI / 2), 0.4);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 600);

        robot.drivetrain.goToPoint(new Pose2d(0, 44, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }

    private void intake_far() {
        robot.drivetrain.goToPoint(new Pose2d(40, 62, Math.toRadians(60)), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);


        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(58, 62, Math.toRadians(60)), 0.5);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 600);

        robot.drivetrain.goToPoint(new Pose2d(50, 50, Math.PI / 2), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
        robot.shooter.reqAim(true);

    }

}
