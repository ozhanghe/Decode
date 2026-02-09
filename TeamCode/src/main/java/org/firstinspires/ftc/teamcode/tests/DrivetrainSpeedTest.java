package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous(name = "DrivetrainSpeedTest")
public class DrivetrainSpeedTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        while(opModeInInit()){
            robot.update();
            telemetry.addData("X-Pos: ", String.valueOf(ROBOT_POSITION.x));
            telemetry.addData("Y-Pos: ", String.valueOf(ROBOT_POSITION.y));
        }

        ElapsedTime ElapsedTime = new ElapsedTime();

        ElapsedTime.reset();

        robot.waitWhile(this::reachedPosition);

        robot.drivetrain.goToPoint(new Pose2d(36, 0, 0), 1);

        robot.waitWhile(this::reachedPosition);

        double time = ElapsedTime.milliseconds() / 1000;

        TelemetryUtil.packet.put("Time to move 36 in forward is: ", time);

        robot.update();

        telemetry.addData("Time to move 36 in: ", String.valueOf(time));

        telemetry.update();

        robot.waitWhile(this::isStopRequested);
    }

    public boolean reachedPosition() {
        return ROBOT_POSITION.x < 36;
    }
}