package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class MergeLocalizer extends Localizer{
    public MergeLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(72, -160, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;
    private Pose2d currPinpointPose = null, lastPinpointPose = null;
    private double lastPinpointUpdate;
    private boolean constantCorrection = false;

    // Limelight
    private LLResult result = null;
    private boolean limelightToggle = false;
    private double lastStaleness = 100.0;

    private final Pose2d redTag = new Pose2d(-58.3414795, 55.6424675);
    private final Pose2d blueTag = new Pose2d(-58.3414795, -55.6424675);
    private final double tagHeight = 29.5;

    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime - lastTime)/1.0E9;
        lastTime = currentTime;

        // 3 WHEEL ODOMETRY

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        Pose2d relDelta = new Pose2d (relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime, relDelta, currentPose);

        // PINPOINT
        if (currentPose.getDistanceFromPoint(currPinpointPose) >= 24.0 || constantCorrection) {
            pinpoint.update();
            double timeStamp = System.nanoTime();
            lastPinpointPose = currPinpointPose.clone();
            currPinpointPose = new Pose2d (pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());

            // find closest pose to last poll time
            int index = 0;
            while (index < nanoTimes.size() && nanoTimes.get(index) - lastPinpointUpdate > 0) {
                index++;
            }

            if (index == nanoTimes.size()){
                index = nanoTimes.size()-1; // death to 10000000000 children
            }

            Pose2d globalPinpointDelta = new Pose2d (
                    currPinpointPose.x - lastPinpointPose.x,
                    currPinpointPose.y - lastPinpointPose.y,
                    currPinpointPose.heading - lastPinpointPose.heading
            );

            Pose2d relPinpointDelta = new Pose2d (
                    Math.cos(lastPinpointPose.heading) * globalPinpointDelta.x + Math.sin(lastPinpointPose.heading) * globalPinpointDelta.y,
                    -Math.sin(lastPinpointPose.heading) * globalPinpointDelta.x + Math.cos(lastPinpointPose.heading) * globalPinpointDelta.y,
                    globalPinpointDelta.heading
            );

            Pose2d globalPinpointEstimate = new Pose2d (
                    poseHistory.get(index).x + Math.cos(poseHistory.get(index).heading) * relPinpointDelta.x - Math.sin(poseHistory.get(index).heading) * relPinpointDelta.y,
                    poseHistory.get(index).y + Math.sin(poseHistory.get(index).heading) * relPinpointDelta.x + Math.cos(poseHistory.get(index).heading) * relPinpointDelta.y,
                    poseHistory.get(index).heading + relPinpointDelta.heading
            );

            currentPose = globalPinpointEstimate.clone();
            lastPinpointUpdate = timeStamp;
        }

        // LIMELIGHT

        if (limelightToggle) {
            drivetrain.vision.update();
            result = drivetrain.vision.getResult();

            if (result != null && result.isValid() && result.getStaleness() < lastStaleness) {
                int index = 0;
                while (index < Shooter.nanoTimes.size() && Shooter.nanoTimes.get(index) - result.getStaleness() > 0) {
                    index++;
                }
            }
        }

        // COMPUTE
        constAccelMath.calculate(loopTime, relDelta, currentPose);
        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public void setPoseEstimate(Pose2d pose) {
        super.setPoseEstimate(pose);
        pinpoint.setPosition(new Pose2D (DistanceUnit.INCH, pose.x, pose.y, AngleUnit.RADIANS, pose.heading));
        currPinpointPose = pose.clone();
        currPinpointPose = pose.clone();
        lastPinpointUpdate = System.currentTimeMillis();
    }

    public void setConstantPinpoint (boolean toggle) { constantCorrection = toggle; }

    public void setLimelightToggle (boolean toggle) { limelightToggle = toggle; }
}
