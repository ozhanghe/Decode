package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDWrapper {
    public final DigitalChannel digitalChannel;
    private boolean newState = false, lastState = false;

    public LEDWrapper(HardwareMap hardwareMap, String name) {
        this.digitalChannel = hardwareMap.get(DigitalChannel.class, name);
        this.digitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
        this.digitalChannel.setState(true);
    }

    public void set(boolean on) {
        this.newState = on;
    }

    public void update() {
        if (this.newState != this.lastState) {
            // Output pin is inverted
            this.digitalChannel.setState(!this.newState);
        }
        this.lastState = this.newState;
    }
}
