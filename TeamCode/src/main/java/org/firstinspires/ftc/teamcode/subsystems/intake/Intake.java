package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Intake {
    private final Robot robot;
    public final PriorityMotor rollerMotor;
    public final PriorityCRServo feedServo;

    public enum State {
        IDLE,
        INTAKE,
        SORT_FEED,
        SORT_WAIT,
        SHOOT_FEED,
        SHOOT_WAIT,
    }

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;

        rollerMotor = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "roller")},
            "roller", 2, 5,
            new double[] {1}, robot.sensors
        );

        feedServo = new PriorityCRServo(
            new CRServo[] {robot.hardwareMap.get(CRServo.class, "feed1"), robot.hardwareMap.get(CRServo.class, "feed2")},
            "feed",
            1, 5, new boolean[] {false, true}
        );

        robot.hardwareQueue.addDevices(rollerMotor, feedServo);
    }

    public void update() {
        switch (state) {
            case IDLE: {
                // TODO Not spinning roller

                // TODO Disable Color Detection?
                break;
            }
            case INTAKE: {
                // TODO Spin roller

                // TODO Toggle Color Detection?
                break;
            }
            case SORT_FEED: {
                // TODO Hit 1 ball into shooter

                // TODO: Toggle Color Detection?
                break;
            }
            case SORT_WAIT: {
                // TODO Buffer state
                break;
            }
            case SHOOT_FEED: {
                // TODO Shoot ball

                // TODO Disable Color Detection?
                break;
            }
            case SHOOT_WAIT: {
                // TODO Buffer state
                break;
            }
        }

        this.updateTelemetry();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", this.state);
        LogUtil.intakeState.set(this.state.toString());
    }
}
