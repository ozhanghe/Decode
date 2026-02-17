package org.firstinspires.ftc.teamcode.subsystems.park;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;


@Config
public class Park {
    private final Robot robot;

    private final PriorityCRServo park;


    public static double forcePullInPower = -0.2;
    public static double stayUpPower = 0.2;

    private double power = 0;



    public Park(Robot robot) {
        this.robot = robot;

        park = new PriorityCRServo(
            new CRServo[]{robot.hardwareMap.get(CRServo.class, "park1"), robot.hardwareMap.get(CRServo.class,"park2")},
            "park", PriorityCRServo.ServoType.AXON_MAX,
            new boolean[]{false, true},
            2, 5, false
        );

        robot.hardwareQueue.addDevice(park);
    }

    public void update() {
        park.setTargetPower(power);
        this.updateTelemetry();

    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Park : power", this.power);

    }

    public void setPower(double p) {
        power = p;
    }
}
