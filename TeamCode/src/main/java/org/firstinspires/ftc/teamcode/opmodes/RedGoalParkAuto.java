package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "RedGoalParkAuto")
public class RedGoalParkAuto extends LinearOpMode {
    private Robot robot;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-72 + ROBOT_LENGTH / 2, 48 - ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.TEST;

        while (opModeInInit()) { robot.update(); }

        robot.drivetrain.goToPoint(new Pose2d(-32, 52, 0), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        do { AUTO_ENDING_POSE = ROBOT_POSITION.clone(); } while (!isStopRequested());
    }
}
