package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Deposit {
    public enum State {
        IDLE, // Linear slides down and Bucket in default position
        RAISE, // Raises up
        DUMP, // Dumps ball
        LOWER, // Lowers back to IDLE position,
        TEST
    }

    public State state = State.IDLE;
    private final Robot robot;
    public final PriorityMotor slides;
    public final nPriorityServo bucketArmServos; // Controls arm connected to bucket
    public final nPriorityServo bucketServo; // This one controls the bucket
    public boolean requestRaise = false, requestDump = false, requestDown = false;

    public Deposit(Robot robot) {
        this.robot = robot;

        slides = new PriorityMotor(
                new DcMotorEx[]{
                        robot.hardwareMap.get(DcMotorEx.class, "slide1"),
                        robot.hardwareMap.get(DcMotorEx.class, "slide2")
                },
                "slides", 3, 5, new double[]{1, -1},
                robot.sensors
        );

        bucketArmServos = new nPriorityServo( // TODO: Numbers need to be calibrated (these are guesses)
                new Servo[]{
                        robot.hardwareMap.get(Servo.class, "bucketArmServo1"),
                        robot.hardwareMap.get(Servo.class, "bucketArmServo2")
                },
                "bucketArm", nPriorityServo.ServoType.AXON_MINI,
                0, 0.5, 0, new boolean[]{false}, 2, 3
        );

        bucketServo = new nPriorityServo( // TODO: Numbers need to be calibrated
                new Servo[]{
                        robot.hardwareMap.get(Servo.class, "bucketServo1")
                },
                "bucket", nPriorityServo.ServoType.AXON_MINI,
                0, 0.5, 0, new boolean[]{false}, 2, 3
        );


        robot.hardwareQueue.addDevices(slides);
        robot.hardwareQueue.addDevices(bucketArmServos);
        robot.hardwareQueue.addDevices(bucketServo);

    }
    public void update() {
        switch (this.state) {
            case IDLE: {
                if (this.requestRaise) {
                    this.state = State.RAISE;
                }

                break;
            }
            case RAISE: {
                // TODO: Implement RAISE state
                break;
            }
            case DUMP: {
                // TODO: Implement DUMP state
                break;
            }
            case LOWER: {
                // TODO: Implement LOWER state
                this.slides.setTargetPower(-1);
                break;
            }
            case TEST: {
                break;
            }


        }
    }
}



