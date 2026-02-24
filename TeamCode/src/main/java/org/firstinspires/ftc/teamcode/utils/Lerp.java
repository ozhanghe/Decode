package org.firstinspires.ftc.teamcode.utils;

public class Lerp {

    public static double lerp(double a, double b, double t) {
        return a * (t) + b * (1 - t);
    }

    public static double lerpAngle(double a, double b, double t) {
        double d = ((b - a + Math.PI) % Math.PI * 2) - Math.PI; //finds the shortest path
        return a + t * d;
    }
}
