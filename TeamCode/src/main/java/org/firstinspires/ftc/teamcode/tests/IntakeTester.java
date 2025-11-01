package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

@Config
@TeleOp(group = "IntakeTester")
public class IntakeTester extends LinearOpMode {
    private Robot robot;
    private Intake intake;

    public static double feedServoPower = 0.0, intakeMotorPower = 0.0;
    public static boolean updateValues = false;

    public void runOpMode(){
        robot = new Robot(hardwareMap);
        intake = robot.intake;

        while(opModeInInit()){
            robot.update();
        }

        // TODO: Add Color Sensor once wired
        while(opModeIsActive()){
            if(updateValues){
                intake.feedServo.setTargetPower(feedServoPower);
                intake.rollerMotor.setTargetPower(intakeMotorPower);
                updateValues = !updateValues;
            }
        }
    }
}
