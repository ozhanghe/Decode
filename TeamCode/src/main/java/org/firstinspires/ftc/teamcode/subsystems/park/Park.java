package org.firstinspires.ftc.teamcode.subsystems.park;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

@Config
public class Park {
    private final PriorityCRServo park;

    private double power = 0;

    public Park(Robot robot) {
        park = new PriorityCRServo(
            new CRServo[]{robot.hardwareMap.get(CRServo.class, "park1"), robot.hardwareMap.get(CRServo.class, "park2")},
            "park", PriorityCRServo.ServoType.AXON_MAX,
            new boolean[]{false, true},
            2, 2
        );

        robot.hardwareQueue.addDevice(park);
    }

    public void update() {
        park.setTargetPower(power);
        TelemetryUtil.packet.put("Park : power", this.power);
    }

    public void setPower(double p) {
        power = p;
    }
}
