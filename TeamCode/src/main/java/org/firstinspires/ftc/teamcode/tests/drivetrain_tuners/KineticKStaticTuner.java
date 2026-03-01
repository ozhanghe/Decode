package org.firstinspires.ftc.teamcode.tests.drivetrain_tuners;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.Locale;

@Autonomous(group = "Test", name = "Kinetic Kstatic Tuner")
@Config
public class KineticKStaticTuner extends LinearOpMode {
    public static double scalar = 0.005;

    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.state = Drivetrain.State.IDLE;
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
        robot.shooter.state = Shooter.State.TEST;

        /*robot.drivetrain.rightRear.setMinimumPowerToOvercomeFriction(0);
        robot.drivetrain.rightFront.setMinimumPowerToOvercomeFriction(0);
        robot.drivetrain.leftRear.setMinimumPowerToOvercomeFriction(0);
        robot.drivetrain.leftFront.setMinimumPowerToOvercomeFriction(0); ya use this later ndjasduias h*/

        robot.drivetrain.resetMinPowersToOvercomeFriction();
        waitForStart();

        double power = 0.0;

        while (opModeIsActive()) {
            double velx = ROBOT_VELOCITY.x;

            power += scalar * (velx > 5.0 ? -1 : 1);

            robot.drivetrain.setMoveVector(new Vector2(power, 0), 0);
            Log.i("Power", String.format(Locale.US, "%.3f, velx %.3f", power, velx));

            robot.waitFor(50);
        }
    }
}
