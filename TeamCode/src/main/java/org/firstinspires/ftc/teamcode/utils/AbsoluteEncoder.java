package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AbsoluteEncoder {
    //private String name;
    private final AnalogInput encoder;
    private double currentAngle;
    private double prevAngle = 0;
    private double angleTraveled = 0;

    public AbsoluteEncoder(HardwareMap hardwareMap, String name) {
        encoder = hardwareMap.get(AnalogInput.class, name);
        prevAngle = currentAngle = normalizeVoltage(encoder.getVoltage());
    }

    /**
     * Converts an analog voltage to an angle
     * @param v voltage [0,3.3]
     * @return angle [0,2PI]
     */
    public static double normalizeVoltage(double v) {
        return (v / 3.3) * 2.0 * Math.PI;
    }

    public void update() {
        prevAngle = currentAngle;
        currentAngle = normalizeVoltage(encoder.getVoltage());
        angleTraveled += Utils.headingClip(currentAngle - prevAngle);
        prevAngle = currentAngle;
    }

    public double getAngleTraveled() { return angleTraveled; }
    public void setAngleTraveled(double angle) { angleTraveled = angle; }
}
