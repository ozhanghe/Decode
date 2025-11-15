package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(group = "Test")
public class ShooterTester extends LinearOpMode {
    public static double turretAngle = 0.0, hoodAngle = 0.7, flywheelVelocity = 0.0, rollerPower = 0.7, feedPower = 0.5;
    public static boolean updateVelocity = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;
        robot.intake.state = Intake.State.TEST;

        ButtonToggle up = new ButtonToggle();
        ButtonToggle down = new ButtonToggle();

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            if (gamepad1.x) robot.intake.feed.setTargetPower(-feedPower);
            else if (gamepad1.y) robot.intake.feed.setTargetPower(feedPower);
            else robot.intake.feed.setTargetPower(0);
            if (gamepad1.a) robot.intake.roller.setTargetPower(-rollerPower);
            else if (gamepad1.b) robot.intake.roller.setTargetPower(rollerPower);
            else robot.intake.roller.setTargetPower(0);

            if (up.isClicked(gamepad1.dpad_up)) {
                flywheelVelocity += 5;
                updateVelocity = true;
            }
            if (down.isClicked(gamepad1.dpad_down)) {
                flywheelVelocity -= 5;
                updateVelocity = true;
            }

            shooter.setTurretAngle(turretAngle);
            shooter.setHoodAngle(hoodAngle);

            if (updateVelocity) {
                shooter.setTargetVelocity(flywheelVelocity);
                updateVelocity = false;
            }

            robot.update();
        }
    }
}
