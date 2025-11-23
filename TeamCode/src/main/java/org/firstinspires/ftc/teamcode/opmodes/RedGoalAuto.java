package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Autonomous(name = "RedGoalAuto", preselectTeleOp = "A.Teleop")
public class RedGoalAuto extends LinearOpMode {
    Robot robot;
    long shooterTimer;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(-72.0 + ROBOT_LENGTH / 2, 48 - ROBOT_WIDTH / 2, 0));

        while(opModeInInit()){
            robot.update();
        }

        while(!isStopRequested()){
            robot.drivetrain.goToPoint(new Pose2d(-36, 36, Math.PI * 3/4), 0.5);
            robot.shooter.setShooter(Shooter.State.CLOSE);
            robot.shooter.setShooterBlocker(true);
            robot.update();
            robot.waitWhile(() -> robot.drivetrain.state == Drivetrain.State.WAIT && robot.shooter.atVel());

            shooterTimer = System.currentTimeMillis();
            robot.intake.roller.setTargetPower(0.8);
            robot.intake.feed.setTargetPower(0.8);
            robot.shooter.setShooterBlocker(false);
            robot.update();
            robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 500);

            robot.shooter.setShooter(Shooter.State.OFF);
            robot.shooter.setShooterBlocker(false);
            robot.intake.roller.setTargetPower(0.0);
            robot.intake.feed.setTargetPower(0.0);
            robot.drivetrain.goToPoint(new Pose2d(-12, 24, Math.PI / 2), 0.5);
            robot.update();
        }
    }
}
