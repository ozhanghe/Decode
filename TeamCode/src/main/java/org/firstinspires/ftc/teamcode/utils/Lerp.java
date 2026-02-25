package org.firstinspires.ftc.teamcode.utils;

public class Lerp {

    public static double lerp(double a, double b, double t) {
        return a * (1 - t) + b * t;
    }

    public static double lerpAngle(double a, double b, double t) {
        double d = (b - a + Math.PI) % (2 * Math.PI) - Math.PI; //finds the shortest path
        return a + t * d;
    }
}
