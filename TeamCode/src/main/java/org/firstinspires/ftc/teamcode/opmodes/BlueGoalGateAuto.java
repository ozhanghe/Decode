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
@Autonomous(name = "* Blue Goal Gate Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class BlueGoalGateAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 850, intakeDuration = 1900, gateIntakeDuration = 1500, gateOpenDuration = 600;

    public void runOpMode() {
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-71 + ROBOT_BACK_LENGTH, -24.25 - ROBOT_WIDTH / 2, 0));
        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0P.set(System.currentTimeMillis() % 500 < 350);
        }
        robot.sensors.light0P.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        //robot.drivetrain.goToPoint(new Pose2d(-40, 40, 0), 1.0);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.shooter.setShooter(Shooter.Dist.MID);
        robot.shooter.turretTrackInManual = true;

        long t = System.currentTimeMillis();
        //robot.shooter.reqAim(true);
        shoot(-Math.PI / 4, 0);
        intake(11, -60);
        shoot(-Math.PI / 4, 1);
        gate_intake();
        shoot(-Math.PI / 2, 1);
        intake(-12, -53);
        shoot(-Math.PI, 1);
        robot.shooter.setShooter(Shooter.Dist.CLOSE);
        robot.drivetrain.goToPoint(new Pose2d(18, -22, -Math.PI), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        intake(36, -60);
        shoot(-Math.PI, 2);

        robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.targetTurretAngle = 0.0;
        //robot.drivetrain.goToPoint(new Pose2d(0, -40, -Math.PI / 2), 1.0);

        long x = System.currentTimeMillis() - t;

        TelemetryUtil.packet.put("Time : ", x);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading, int shotType) {
        if (shotType > 0) {
            robot.drivetrain.goToPoint(new Pose2d(shotType == 2 ? -32 : -13, -20, heading), 0.8, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }

        robot.drivetrain.goToPoint(new Pose2d(shotType == 2 ? -40 : -18, -18, heading), 0.4);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel() || Math.abs(robot.shooter.targetTurretAngle - robot.sensors.getTurretAngle()) > 3);
        //robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
        robot.waitFor(shotType == 0 ? 600 : 400);

        //robot.shooter.reqShoot(true);
        robot.shooter.setShooterBlocker(false);
        robot.intake.reqShoot(true);
        robot.waitFor(shootDuration);
        robot.shooter.setShooterBlocker(true);
        robot.intake.reqOff(true);

        //robot.shooter.reqStop(true);
        //if (recharge) robot.shooter.reqAim(true);
    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, -22, -Math.PI / 2), 0.5);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        robot.drivetrain.goToPoint(new Pose2d(x, y, -Math.PI / 2), 0.25);
        robot.waitFor(intakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(x, -45, -Math.PI / 2), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqOff(true);
    }

    private void open_gate(long duration) {
        robot.drivetrain.goToPoint(new Pose2d(0, -48, -Math.PI / 2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.drivetrain.goToPoint(new Pose2d(0, -53, -Math.PI / 2), 0.5);
        robot.waitFor(duration);

        robot.drivetrain.goToPoint(new Pose2d(0, -30, -Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }

    private void gate_intake() {
        robot.drivetrain.goToPoint(new Pose2d(5, -35, -Math.PI / 2), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        //open gate
        robot.intake.reqIntake(true);
        robot.drivetrain.goToPoint(new Pose2d(5,-55, -Math.toRadians(118)), 0.7, true);
        robot.waitFor(gateOpenDuration);

        //go behind the gate to intake the balls
        //start farther from the gate
        robot.drivetrain.goToPoint(new Pose2d(20, -63, -Math.toRadians(170)), 0.8);
        robot.waitFor(gateIntakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(15, -36, -Math.toRadians(180)), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
    }
}
