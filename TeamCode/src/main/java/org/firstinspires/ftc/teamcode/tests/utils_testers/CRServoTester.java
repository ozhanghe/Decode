package org.firstinspires.ftc.teamcode.tests.utils_testers;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class CRServoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);
        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<PriorityCRServo> crservos = new ArrayList<>();

        // buttons for changing servo
        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();

        int servoSize = 0;
        int servoIndex = 0;

        // getting number of servos we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            Log.i("HardwareQueue devices", device.name);
            if (device instanceof PriorityCRServo) {
                crservos.add((PriorityCRServo) device);
                servoSize++;
            }
        }

        waitForStart();

        while (!isStopRequested()) {
            START_LOOP();

            if (buttonY.isClicked(gamepad1.y)) {
                crservos.get(servoIndex).setTargetPower(0.0);
                servoIndex++;
            }
            if (buttonA.isClicked(gamepad1.a)) {
                crservos.get(servoIndex).setTargetPower(0.0);
                servoIndex--;
            }

            double servoPower = gamepad1.right_stick_x;
            servoIndex = (servoIndex + servoSize) % servoSize;
            crservos.get(servoIndex).setTargetPower(servoPower);

            robot.sensors.update();
            robot.hardwareQueue.update();

            telemetry.addData("servo index", servoIndex);
            telemetry.addData("servo name", crservos.get(servoIndex).name);
            telemetry.addData("servo power", servoPower);
            telemetry.addData("Turret angle (deg)", Math.toDegrees(robot.sensors.getTurretAngle()));
            telemetry.update();

            TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
            TelemetryUtil.sendTelemetry();
        }
    }
}
