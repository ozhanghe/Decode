package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class NewShooter {

    public enum State {
        IDLE,
        SPINUP,
        READY,
        SHOOT,
        TEST, // empty for debugging
    }

    public State state = State.IDLE;

    private final Robot robot;
    public final Flywheel flywheel;
    public final nPriorityServo blocker;

    public boolean requestShoot, requestStop;

    public static double flywheelTargetVel = 400;
    public static double blockerOpen = 50;
    public static double blockerClose = 20;

    public NewShooter(Robot robot) {
        this.robot = robot;

        this.flywheel = new Flywheel(robot);

        blocker = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "flywheelBlocker")},
                "flywheelBlocker", nPriorityServo.ServoType.AXON_MICRO,
                0, 0.7, 0.1, new boolean[]{false}, 2, 2);

        robot.hardwareQueue.addDevices(blocker);
    }

    public void update() {
        switch (this.state) {
            case IDLE: {
                this.requestStop = false;
                this.flywheel.setTargetVelocity(0);
                this.blocker.setTargetAngle(blockerClose);

                if (this.requestShoot) {
                    this.requestShoot = false;
                    this.state = State.SPINUP;
                }

                break;
            }
            case SPINUP: {
                this.flywheel.setTargetVelocity(flywheelTargetVel);
                this.blocker.setTargetAngle(blockerClose);

                if (this.flywheel.atVel()) { // up to speed
                    this.state = State.READY;
                }

                if (this.requestStop) {
                    this.requestStop = false;
                    this.state = State.IDLE;
                }

                break;
            }
            case READY: {
                this.flywheel.setTargetVelocity(flywheelTargetVel);
                this.blocker.setTargetAngle(blockerClose);

                if (this.requestShoot) {
                    this.requestShoot = false;
                    this.state = State.SHOOT;
                }

                if (this.requestStop) {
                    this.requestStop = false;
                    this.state = State.IDLE;
                }

                break;
            }
            case SHOOT: {
                this.flywheel.setTargetVelocity(flywheelTargetVel);
                this.blocker.setTargetAngle(blockerOpen); // release the ball

                if (this.requestStop) {
                    this.requestStop = false;
                    this.state = State.IDLE;
                }

                break;
            }
            case TEST: {
                break;
            }
        }

        flywheel.update();

        updateTelemetry();
    }

    public void reqShoot(boolean req) {
        this.requestShoot = req;
    }

    public void reqStop(boolean req) {
        this.requestStop = req;
    }

    public void setTest(boolean req) {
        if (req) {
            this.state = State.TEST;
        } else {
            this.state = State.IDLE;
        }
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("NewShooter : state", this.state);
    }
}
