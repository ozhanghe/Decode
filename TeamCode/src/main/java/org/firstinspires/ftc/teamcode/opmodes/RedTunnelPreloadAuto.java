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

@Config
@Autonomous(name = "Red Tunnel Preload Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class RedTunnelPreloadAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 1000, intakeDuration = 2000;

    public static double heading = 2.71;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(71 - ROBOT_BACK_LENGTH, 24.25 - ROBOT_WIDTH / 2, Math.PI));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 250);
        }
        robot.sensors.light0G.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        robot.shooter.setShooter(Shooter.Dist.FAR);
        robot.shooter.turretTrackInManual = true;

        shoot(heading, true);
        intake(35, 60);
        shoot(heading, false);
        intake(11, 60);
        shoot(heading, false);
        intake(-13, 55);
        shoot(heading, false);

        robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(0, 40, Math.PI / 2), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading, boolean firstShot) {
        robot.drivetrain.goToPoint(new Pose2d(53, 12, heading), 0.4);
        robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel() || Math.abs(robot.shooter.targetTurretAngle - robot.sensors.getTurretAngle()) > 2);
        robot.waitFor(firstShot ? 700 : 500);

        robot.shooter.setShooterBlocker(false);
        robot.intake.reqShoot(true);
        robot.waitFor(shootDuration);
        robot.shooter.setShooterBlocker(true);
        robot.intake.reqOff(true);
    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 0.5);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 0.25);
        robot.waitFor(intakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(x, 45, Math.PI / 2), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqOff(true);
    }
}
