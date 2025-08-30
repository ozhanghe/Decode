package org.firstinspires.ftc.teamcode.tests.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SlideEncoderTester extends LinearOpMode{
    public void runOpMode(){
        /*Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        double motorPower = 0.0;

        waitForStart();

        while(!isStopRequested()){
            START_LOOP();
            robot.drivetrain.resetMinPowersToOvercomeFriction();

            if (gamepad1.b) {
                motorPower += 0.01;
            }

            if (gamepad1.x) {
                motorPower -= 0.01;
            }

            if (gamepad1.right_trigger > 0.1) {
                motorPower = 0;
            }

            motorPower = Utils.minMaxClip(motorPower, -1.0, 1.0);
            robot.deposit.slides.slidesMotors.setTargetPower(motorPower);

            robot.sensors.update();
            robot.hardwareQueue.update();

            telemetry.addData("Slides position", sensors.getSlidesPos());
            telemetry.addData("motor power", motorPower);
            telemetry.update();

            TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
            TelemetryUtil.sendTelemetry();
        }*/
    }
}
