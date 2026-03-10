package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Turret {
    private final Robot robot;
    public final nPriorityServo turret;

    public static PID turretPID = new PID (0.15, 0, 0.02);
    public static PID finalAdjustPID = new PID (0.08, 0.0, 0.004);
    public static double turretKStaticBig = 0.07;
    public static double turretKStaticSmall = 0.085;

    public static double turretDeadzoneDeg = 2.5;
    public static double inPositionThresh = Math.toRadians(3);

    public static double turretVelFactor = 0.15;
    private double lastTurretTarget = 0.0;
    private double targetTurretAngle = 0.0;
    private double targetTurretAngleVel = 0.0;
    public static double targetTurretAngleVelFilter = 0.9;

    public Turret(Robot robot) {
        this.robot = robot;

        turret = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class, "turret2")},
                "turret", nPriorityServo.ServoType.AXON_MINI,
                0, 1, 0.92,
                new boolean[] {false, false},
                5, 6
        );
        /*

        turret.setTargetPower(0.1);
        turret.update();
        turret.setTargetPower(0.0);
        turret.update();

         */

        robot.hardwareQueue.addDevice(turret);
    }

    public void update() {

        /*
        // Turret PIDF
        targetTurretAngleVel = targetTurretAngleVel * (1 - targetTurretAngleVelFilter) + (targetTurretAngle - lastTurretTarget) / robot.sensors.loopTime * targetTurretAngleVelFilter;
        targetTurretAngleVel = Utils.minMaxClip(targetTurretAngleVel, -150, 150);
        lastTurretTarget = targetTurretAngle;

        double turretAngle = robot.sensors.getTurretAngle();
        double turretError = targetTurretAngle - turretAngle;
        double turretPow = (Math.abs(turretError) > Math.toRadians(10) ? turretPID.update(turretError, -1, 1): finalAdjustPID.update(turretError, -0.5, 0.5));

        if (Math.abs(turretError) < Math.toRadians(turretDeadzoneDeg)) {
            finalAdjustPID.resetIntegral();
            turretPow = 0;
        }
        if(targetTurretAngleVel > 0.05){
            turretPow += targetTurretAngleVel / (turret.servoType.speed) * turretVelFactor; // meant to account for robot rotating
        }
        if (Math.abs(turretError) > 10) {turretPID.resetIntegral(); finalAdjustPID.resetIntegral();}
        turretPow += (Math.abs(turretError)>10 ? (Math.signum(turretPow) * turretKStaticBig) : (Math.signum(turretPow) * turretKStaticSmall));

        if (Math.abs(turretError) > Math.toRadians(75)) turretPow = Math.signum(turretError);
        if (turretAngle >= Sensors.turretLimitLeft) turretPow = Math.min(turretPow, -turretKStaticBig);
        if (turretAngle <= Sensors.turretLimitRight) turretPow = Math.max(turretPow, turretKStaticBig);



        turretPow = Utils.minMaxClip(turretPow, -1, 1);
        turret.setTargetPower(turretPow);

        updateTelemetry(turretPow, turretError);

         */

        turret.setTargetAngle(targetTurretAngle);

        updateTelemetry();
    }

    public void setTargetAngle(double targetAngle) {
        //positive turret limit in radians
        if(targetAngle > 0.42586033761) {
            while(targetAngle > 0.42586033761) {
                targetAngle -= Math.toRadians(305);
            }
            //negative turret limit in radians
        } else if(targetAngle < -4.89739388258) {
            while(targetAngle < -4.89739388258) {
                targetAngle += Math.toRadians(305);
            }
        }

        targetTurretAngle = targetAngle * -1.0;
    }

    public double getTargetAngle() { return targetTurretAngle; }

    public double getCurrentAngle() {return turret.getCurrentAngle();}

    public boolean inPosition() { return turret.inPosition(); }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Turret : pos", Math.toDegrees(turret.getCurrentAngle()));
        TelemetryUtil.packet.put("Turret : target", Math.toDegrees(turret.getTargetAngle()));


        LogUtil.turretTarget.set(targetTurretAngle);
    }
}
