package org.firstinspires.ftc.teamcode.tests.utils_testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class RightFrontMotorTester extends LinearOpMode {

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        DcMotorEx m1 = robot.hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx m2 = robot.hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx m3 = robot.hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx m4 = robot.hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx m5 = robot.hardwareMap.get(DcMotorEx.class, "roller");
        DcMotorEx m6 = robot.hardwareMap.get(DcMotorEx.class, "feed");

        waitForStart();
        while (!isStopRequested()) {
            m1.setPower(gamepad1.y ? 0.5 : 0);
            m2.setPower(gamepad1.a ? 0.5 : 0);
            m3.setPower(gamepad1.x ? 0.5 : 0);
            m4.setPower(gamepad1.b ? 0.5 : 0);
            m5.setPower(gamepad1.left_bumper ? 0.5 : 0);
            m6.setPower(gamepad1.right_bumper ? 0.5 : 0);
        }
    }
}
