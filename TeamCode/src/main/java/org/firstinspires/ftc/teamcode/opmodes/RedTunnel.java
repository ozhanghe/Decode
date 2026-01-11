package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

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

@Autonomous(name = "Red Tunnel")
public class RedTunnel extends LinearOpMode {
    long delay;
    public static long shootDuration = 3000;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        // Location of turret center
        robot.drivetrain.setPoseEstimate(new Pose2d(66, 18, Math.PI));

        while (opModeInInit()) {
            robot.sensors.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        // Preload
        robot.shooter.reqAim(true);
        Path path = new Path(new Pose2d(54, 18, Math.PI), Globals.getMidline())
                .setDecel(true);
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < shootDuration);

        robot.shooter.reqStop(true);
        robot.update();

        // Tunnel side spikes
        robot.intake.reqIntake(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(36, 30, Math.PI * 0.5))
                .addPoint(new Pose2d(36, 37.5, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
        robot.shooter.reqAim(true);
        path = new Path(ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(54, 18, Math.PI));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < shootDuration);

        robot.shooter.reqStop(true);
        robot.update();

        // Park
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(48, 24, 0));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
