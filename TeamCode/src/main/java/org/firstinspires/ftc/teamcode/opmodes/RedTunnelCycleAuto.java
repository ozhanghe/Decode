package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@Autonomous(name = "RedTunnelCycleAuto", group = "Auto", preselectTeleOp = "A. Teleop")
public class RedTunnelCycleAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 850, intakeDuration = 1900;
    private double timer = System.currentTimeMillis();
    private final double x = 60;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(71 - ROBOT_BACK_LENGTH, 24.25 - ROBOT_WIDTH / 2, Math.PI));
        robot.shooter.state = Shooter.State.IDLE;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 350);
        }
        robot.sensors.light0G.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        shoot();
        intake();
        shoot();
        intake();
        shoot();
        intake();
        shoot();
        intake();
        shoot();

        robot.shooter.turret.setTargetAngle(0.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot() {
        robot.shooter.reqAim(true);

        Path path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline()).setDecel(true).addPoint(new Pose2d(x, 15, Math.PI / 2));

        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> {
            robot.shooter.turretTrackTarget();
            return robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY || !robot.shooter.turret.inPosition();
        });

        robot.waitFor(200);

        robot.intake.reqOff(true);

        robot.shooter.reqShoot(true);
        timer = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - timer) < shootDuration);

        robot.shooter.reqStop(true);
        robot.update();
    }

    private void intake() {
        robot.intake.reqIntake(true);

        Path path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline()).setDecel(true).addPoint(new Pose2d(x, 60, Math.PI / 2));

        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.waitFor(800);

        robot.drivetrain.goToPoint(new Pose2d(x-10, 60, Math.PI/2), 0.8);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.waitFor(400);

    }
}
