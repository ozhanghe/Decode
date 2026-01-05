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

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Park park = new Park(robot);
        park.state = Park.State.IDLE;
        double targetAngle = 0;

        while (opModeInInit()) {
            robot.update();
            park.update();
        }


        while (!isStopRequested()) {
            if (gamepad1.x) park.moveNextState();
            if (gamepad1.y) park.movePreviousState();
            if (gamepad1.a) park.state = Park.State.MANUAL_CONTROL;
            if (gamepad2.x && park.state == Park.State.MANUAL_CONTROL) park.setTargetAngle(targetAngle + 0.2); targetAngle += 0.2;
            if(gamepad2.y && park.state == Park.State.MANUAL_CONTROL) park.setTargetAngle(targetAngle - 0.2); targetAngle -= 0.2;





            robot.update();
            park.update();
        }
    }

}
