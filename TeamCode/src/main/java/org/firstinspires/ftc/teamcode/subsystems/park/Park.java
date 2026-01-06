package org.firstinspires.ftc.teamcode.subsystems.park;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

public class Park {
    private final Robot robot;

    private final PriorityCRServo pivot;

    //every turn of the servo leads to the bellypan spinning gear_ratio times
    private static final double gear_ratio = 36.0 / (4.0 * 40.0);

    //bellypan positions
    //horizontal is 0
    private double target_angle = 0;
    private double angle = 0;

    private final double threshold = 0.1; // idk tune
    public static PID bellypanPID = new PID (0.01, 0.0, 0.0);

    public enum State {
        IDLE,
        STAGE_1,
        STAGE_2,
        WAIT,
        MANUAL_CONTROL
    }

    private boolean nextState = false;
    private boolean previousState = false;

    public State state = State.IDLE;

    public Park(Robot robot) {
        this.robot = robot;

        pivot = new PriorityCRServo(
            new CRServo[]{robot.hardwareMap.get(CRServo.class, "park1"), robot.hardwareMap.get(CRServo.class,"park2")},
            "park", PriorityCRServo.ServoType.AXON_MAX,
            new boolean[]{false, true},
            2, 5
        );

        robot.hardwareQueue.addDevice(pivot);
    }

    public void update() {
        //two stages of park
        //stage 1 is we tilt the servos to a certain position(maybe 90 degrees)
        //stage 2 is we tilt the opposite direction

        angle = getBellypanAngle();

        switch (state) {
            case IDLE: {
                setTargetAngle(0);
                if (nextState) {
                    state = State.STAGE_1;
                    nextState = false;
                }
                break;
            }
            case STAGE_1: {
                setTargetAngle(Math.PI / 2);
                if (nextState || Math.abs(angle - Math.PI / 2) < threshold) {
                    state = State.STAGE_2;
                    nextState = false;
                } else if (previousState) {
                    state = State.IDLE;
                    previousState = false;
                }
                break;
            }
            case STAGE_2: {
                setTargetAngle(Math.PI / 4);
                if (nextState || Math.abs(angle - Math.PI / 4) < threshold) {
                    state = State.WAIT;
                    nextState = false;
                } else if (previousState) {
                    state = State.STAGE_1;
                    previousState = false;
                }
            }
            case WAIT: {
                break;
            }
            case MANUAL_CONTROL: {
                //kinda chopped but for now just use the gamepad to manually set targetAngle
                break;
            }
        }

        //angle the bellypan needs to spin and then convert it into the angle the servo needs to spin
        double pow = bellypanPID.update((target_angle - angle) / gear_ratio, -1, 1);
        pivot.setTargetPower(pow);

        this.updateTelemetry();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Park : state", this.state);
        TelemetryUtil.packet.put("Park : angle", this.angle);
        LogUtil.parkState.set(this.state.toString());
        LogUtil.parkAngle.set(this.angle);
    }

    public void setTargetAngle(double target) { target_angle = target; }

    public double getBellypanAngle() { return robot.sensors.getParkAngleTraveled() * gear_ratio; }

    public void moveNextState() { nextState = true; }

    public void movePreviousState() {
        //wtf is the state before idle :sob
        if (this.state != State.IDLE) {
            previousState = true;
        }
    }
}
