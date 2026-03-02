package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.Locale;

@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor roller = hardwareMap.dcMotor.get("roller");
        DcMotor feed = hardwareMap.dcMotor.get("feed");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        roller.setDirection(DcMotor.Direction.REVERSE);



        //back right
        //front left
        //front right
        //back left

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            while (!isStopRequested()) {
                if (gamepad1.x) feed.setPower(0);
                if (gamepad1.y) feed.setPower(1);
                if (gamepad1.a) roller.setPower(0);
                if (gamepad1.b) roller.setPower(1);

        }}
    }


}