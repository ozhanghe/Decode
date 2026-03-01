package org.firstinspires.ftc.teamcode.tests.drivetrain_tuners;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;

@Autonomous(group = "Test")
public class MinimumPowerToOvercomeFrictionDrivetrainTuner extends LinearOpMode {

    double[] sums = new double[2];
    int iterations = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        ArrayList<PriorityMotor> motors = new ArrayList<>();

        double[] minPowersToOvercomeFriction = new double[2];

        motors.add(robot.drivetrain.leftFront);
        motors.add(robot.drivetrain.leftRear);
        motors.add(robot.drivetrain.rightRear);
        motors.add(robot.drivetrain.rightFront);

        Pose2d robotPose;
        robot.drivetrain.resetMinPowersToOvercomeFriction();

        robot.shooter.state = Shooter.State.TEST;

        waitForStart();

        for (int i = 0; i < 2; i++) {
            for (int a = 0; a < iterations; a++) {
                robot.drivetrain.setPoseEstimate(new Pose2d(0,0,0));

                long start = System.currentTimeMillis();
                for (double j = 0; j < 1; j += 0.01) {
                    motors.get(i).setTargetPower(j);
                    motors.get(i + 2).setTargetPower(j);

                    Log.i("Power", String.valueOf(j));
                    telemetry.addData("iteration", a);
                    telemetry.addData("motors", motors.get(i).name + " " +  motors.get(i + 2).name);
                    telemetry.addData("current power", j);
                    telemetry.update();

                    robot.waitFor(70);

                    robotPose = robot.drivetrain.getPoseEstimate();
                    if (Math.abs(robotPose.x) > 0.5 || Math.abs(robotPose.y) > 0.5 || Math.abs(robotPose.heading) > Math.toRadians(5)) {
                        minPowersToOvercomeFriction[i] = j;
                        break;
                    }
                }

                motors.get(i).setTargetPower(0.0);
                motors.get(i + 2).setTargetPower(0.0);

                sums[i] += minPowersToOvercomeFriction[i] * (robot.sensors.getVoltage() / 13.5);

                robot.waitFor(1000);
            }

            Log.i("Power", motors.get(i).name + " " +  motors.get(i + 2).name + " AVERAGE min power with voltage correction " + sums[i]/iterations);

        }
    }
}
