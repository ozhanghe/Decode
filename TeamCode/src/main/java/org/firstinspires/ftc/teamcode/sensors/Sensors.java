package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.RelativeEncoder;
import org.firstinspires.ftc.teamcode.utils.Utils;

@Config
public class Sensors {
    private final Robot robot;

    public double loopTime;
    private long currentTime, lastTime;

    private final int[] odoWheelPositions = {0, 0, 0};

    // Enocder Resolution: 28 PPR
    private double flywheelAngularVel = 0, flywheelVelocity = 0;

    public RelativeEncoder parkEncoder;
    public AnalogInput turretEncoder;
    private double turretEncoderVoltage, turretAngle, lastTurretAngle;
    public static double turretEncoderOffset = Math.toRadians(187);
    public static double turretAngleFilter = 0.4;
    public static double turretLimitLeft = Math.toRadians(120), turretLimitRight = Math.toRadians(-180), turretWrapMid = Math.toRadians(-45);

    private double voltage;
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    public Sensors(Robot robot) {
        this.robot = robot;

        currentTime = System.nanoTime();
        voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();

        //parkEncoder = new RelativeEncoder(robot.hardwareMap, "park_encoder");
        turretEncoder = robot.hardwareMap.get(AnalogInput.class, "turret_encoder");
        lastTurretAngle = turretAngle = 0;
    }

    public void update() {
        lastTime = currentTime;
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastTime) / 1e9;

        odoWheelPositions[0] = robot.drivetrain.leftFront.motor[0].getCurrentPosition();
        odoWheelPositions[1] = robot.drivetrain.rightFront.motor[0].getCurrentPosition();
        odoWheelPositions[2] = robot.drivetrain.leftRear.motor[0].getCurrentPosition();

        double flywheelPos = robot.drivetrain.rightRear.motor[0].getCurrentPosition();
        // (flywheelPos - flywheelLastPos) / 28.0 = delta revolutions
        flywheelAngularVel = robot.drivetrain.rightRear.motor[0].getVelocity() / 28.0;
        flywheelVelocity = flywheelAngularVel * 96.0 * Math.PI / 25.4;

        robot.drivetrain.mergeLocalizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.mergeLocalizer.update();
        ROBOT_POSITION = robot.drivetrain.mergeLocalizer.getPoseEstimate();
        ROBOT_VELOCITY = robot.drivetrain.mergeLocalizer.getRelativePoseVelocity();

        //parkEncoder.update();

        lastTurretAngle = turretAngle;
        turretEncoderVoltage = turretEncoder.getVoltage();
        if (turretEncoderVoltage > 0.1) turretAngle = turretAngle * (1 - turretAngleFilter)
            + (Utils.headingClip(RelativeEncoder.normalizeVoltage(turretEncoderVoltage) - turretEncoderOffset - turretWrapMid) + turretWrapMid) * turretAngleFilter;

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis();
        }

        updateTelemetry();
    }

    public double getFlywheelVelocity() { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    /**
     * Clips a turret angle
     * @param angle a robot-relative angle
     * @return the wrapped and clipped turret angle
     */
    public static double turretAngleClip(double angle) { return Utils.minMaxClip(Utils.headingClip(angle - turretWrapMid) + turretWrapMid, turretLimitRight, turretLimitLeft); }

    /**
     * Gets the turret angle
     * @return the wrapped turret angle
     */
    public double getTurretAngle() { return turretAngle; }

    //angle that the park servo has traveled, not the bellypan
    public double getParkAngleTraveled() { return parkEncoder.getAngleTraveled(); }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Voltage", voltage);
        //TelemetryUtil.packet.put("Shooter : Flywheel Angular Velocity", flywheelAngularVel);
        TelemetryUtil.packet.put("Shooter : Flywheel Current Velocity", flywheelVelocity);
        TelemetryUtil.packet.put("Shooter : Turret angle (deg)", Math.toDegrees(turretAngle));
        TelemetryUtil.packet.put("Shooter : Hood top angle (deg)", Math.toDegrees(robot.shooter.hood.getCurrentAngle()) * 30 / 48 + 34);
        TelemetryUtil.packet.put("Shooter : Turret encoder voltage", turretEncoderVoltage);
        //TelemetryUtil.packet.put("Park : Servo angle", parkEncoder.getAngleTraveled());

        Pose2d currentPose = ROBOT_POSITION;
        TelemetryUtil.packet.put("Robot position", currentPose.toString());
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, "#00ff00", turretAngle, "#008000");

        LogUtil.turretAngle.set(turretAngle);
        LogUtil.flywheelVelocity.set(flywheelVelocity);
        LogUtil.driveCurrentX.set(currentPose.x);
        LogUtil.driveCurrentY.set(currentPose.y);
        LogUtil.driveCurrentAngle.set(currentPose.heading);
    }
}
