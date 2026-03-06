package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

@Config
public class Turret {
    private final Robot robot;
    public final PriorityCRServo turret;

    public static PID turretPID = new PID (0.15, 0, 0.04);
    public static PID finalAdjustPID = new PID (0.02, 0.02, 0.04);
    public static double turretKStatic = 0.12;

    public static double turretDeadzone = Math.toRadians(1.5);
    public static double inPositionThresh = Math.toRadians(3);

    public static double turretVelFactor = 0.2;
    private double lastTurretTarget = 0.0;
    private double targetTurretAngle = 0.0;
    private double targetTurretAngleVel = 0.0;
    public static double targetTurretAngleVelFilter = 0.9;

    public Turret(Robot robot) {
        this.robot = robot;

        turret = new PriorityCRServo(
                new CRServo[] {robot.hardwareMap.get(CRServo.class, "turret1"), robot.hardwareMap.get(CRServo.class, "turret2")},
                "turret", PriorityCRServo.ServoType.AXON_MINI,
                new boolean[] {false, false},
                5, 6
        );

        turret.setTargetPower(0.1);
        turret.update();
        turret.setTargetPower(0.0);
        turret.update();

        robot.hardwareQueue.addDevice(turret);
    }

    public void update() {
        // Turret PIDF
        targetTurretAngleVel = targetTurretAngleVel * (1 - targetTurretAngleVelFilter) + (targetTurretAngle - lastTurretTarget) / robot.sensors.loopTime * targetTurretAngleVelFilter;
        targetTurretAngleVel = Utils.minMaxClip(targetTurretAngleVel, -150, 150);
        lastTurretTarget = targetTurretAngle;

        double turretAngle = robot.sensors.getTurretAngle();
        double turretError = targetTurretAngle - turretAngle;
        double turretPow = (Math.abs(turretError) > Math.toRadians(10) ? turretPID.update(turretError, -1, 1) : finalAdjustPID.update(turretError, -0.5, 0.5)) + (Math.abs(turretError) >= Math.toRadians(5) ? turretKStatic * Math.signum(turretError) : 0);

        if (Math.abs(turretError) < turretDeadzone) {
            finalAdjustPID.resetIntegral();
            turretPow = 0;
        }
        turretPow += targetTurretAngleVel / (turret.servoType.speed) * turretVelFactor; // meant to account for robot rotating

        if (Math.abs(turretError) > Math.toRadians(75)) turretPow = Math.signum(turretError);
        if (turretAngle >= Sensors.turretLimitLeft) turretPow = Math.min(turretPow, -turretKStatic);
        if (turretAngle <= Sensors.turretLimitRight) turretPow = Math.max(turretPow, turretKStatic);

        turretPow = Utils.minMaxClip(turretPow, -1, 1);
        turret.setTargetPower(turretPow);

        updateTelemetry(turretPow, turretError);
    }

    public void setTargetAngle(double targetAngle) { targetTurretAngle = Sensors.turretAngleClip(targetAngle); }

    public double getTargetAngle() { return targetTurretAngle; }

    public boolean inPosition() { return Math.abs(targetTurretAngle - robot.sensors.getTurretAngle()) <= inPositionThresh; }

    private void updateTelemetry(double turretPow, double turretError) {
        TelemetryUtil.packet.put("Turret : targetAngleVel (deg)", Math.toDegrees(targetTurretAngleVel));
        TelemetryUtil.packet.put("Turret : Target Angle (deg)", Math.toDegrees(targetTurretAngle));
        TelemetryUtil.packet.put("Turret : Power Applied", turretPow * 100);
        TelemetryUtil.packet.put("Turret : Error", turretError);
        LogUtil.turretTarget.set(targetTurretAngle);
    }
}
