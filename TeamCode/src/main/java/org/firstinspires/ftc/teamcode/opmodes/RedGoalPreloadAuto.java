package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "RedGoalPreloadAuto")
public class RedGoalPreloadAuto extends LinearOpMode {
    private Robot robot;
    private double shooterTimer = System.currentTimeMillis();

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-72 + Globals.ROBOT_LENGTH / 2, 48 - Globals.ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) { robot.update(); }

        robot.drivetrain.goToPoint(new Pose2d(-40, 40, 0), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.drivetrain.goToPoint(new Pose2d(-34, 34, Math.atan2(Globals.redTag.y - 34, Globals.redTag.x + 34)), 1.0);
        robot.shooter.setShooter(Shooter.Dist.CLOSE);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel());

        robot.shooter.setShooterBlocker(false);
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition(0.1));

        shooterTimer = System.currentTimeMillis();
        robot.intake.reqShoot(true);
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 3000);

        robot.drivetrain.goToPoint(new Pose2d(-32, 52, Math.toRadians(150)), 1.0);
        robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.intake.reqOff(true);

        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
