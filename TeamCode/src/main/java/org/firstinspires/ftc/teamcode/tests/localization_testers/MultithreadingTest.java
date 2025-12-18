package org.firstinspires.ftc.teamcode.tests.localization_testers;

import static org.firstinspires.ftc.teamcode.tests.localization_testers.MultithreadingTest.opModeStartTime;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

class ControlHub implements Runnable {
    DcMotorEx m;
    boolean triggered = false;
    public ControlHub(DcMotorEx m) { this.m = m; }

    public void run() {
        if (System.currentTimeMillis() - opModeStartTime > 5000 && !triggered) {
            int pos = m.getCurrentPosition();
            double time = System.currentTimeMillis() - opModeStartTime;
            Log.i ("THREADING - Control",  time + "");
            triggered = true;
        }
    }
}

class ExpansionHub implements  Runnable {
    DcMotorEx m;
    boolean triggered = false;
    public ExpansionHub(DcMotorEx m) { this.m = m; }

    public void run() {
        if (System.currentTimeMillis() - opModeStartTime > 5000 && !triggered) {
            int pos = m.getCurrentPosition();
            double time = System.currentTimeMillis() - opModeStartTime;
            Log.i ("THREADING - Expansion",  time + "");
            triggered = true;
        }
    }
}

@TeleOp
public class MultithreadingTest extends LinearOpMode {
    public static double opModeStartTime = System.currentTimeMillis();

    public void runOpMode(){
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "motor2");

        ControlHub chub = new ControlHub(m1);
        ExpansionHub exhub = new ExpansionHub(m2);

        while (opModeInInit()){ opModeStartTime = System.currentTimeMillis(); }

        while (!isStopRequested()) {
            chub.run();
            exhub.run();
        }
    }
}
