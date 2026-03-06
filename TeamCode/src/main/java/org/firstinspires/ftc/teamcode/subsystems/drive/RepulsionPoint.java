package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

public class RepulsionPoint extends Pose2d {
    private final double weight;

    public RepulsionPoint (double x, double y, double w) {
        super(x, y);
        weight = w;
    }

    /**
     * This method uses a Gaussian approximation to determine the magnitude of repulsive influence. The weight determines the distance at which the repulsion point begins to have a noticeable effect.
     *
     * @param
     * @return Repulsion Point Influence (Vector2)
     */
    public Vector2 getInfluence(Pose2d robot) {
        Vector2 r = new Vector2 (robot.x - this.x, robot.y - this.y);
        double mag = Math.pow(Math.E, 1 - Math.pow(r.mag()/weight, 2)); // if (r.mag() / weight) ^ 2 is greater than 1 -> too far, the repulsive force is negligible. https://www.desmos.com/calculator/ltsnngnnht
        r.norm();
        r.mul(mag);
        return r;
    }
}
