package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.park.Park;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;


@Config
@TeleOp(group = "Test")
public class ParkTester extends LinearOpMode {
    public static double targetAngle = 0;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Park park = new Park(robot);
        park.state = Park.State.IDLE;
        targetAngle = 0;

        while (opModeInInit()) {
            robot.update();
            park.update();
        }

        while (!isStopRequested()) {
            if (gamepad1.x) park.state = Park.State.IDLE;
            if (gamepad1.y) park.state = Park.State.MANUAL_CONTROL;
            if (park.state == Park.State.MANUAL_CONTROL) {
                if (gamepad2.a) targetAngle += 0.1;
                if (gamepad2.b) targetAngle -= 0.1;
                park.setTargetAngle(targetAngle);
            } else {
                if (gamepad1.a) park.moveNextState();
                if (gamepad1.b) park.movePreviousState();
            }

            robot.update();
            park.update();
        }
    }

}
