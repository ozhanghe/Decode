package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class Deposit {
    private Robot robot;
    private Globals globals;

    private DcMotorEx[] shooters = {robot.hardwareMap.get(DcMotorEx.class, "shooter1"), robot.hardwareMap.get(DcMotorEx.class, "shooter2")};

    private nPriorityServo turret, hood, cloth;
    private PriorityMotor shooter;

    private double[] shooter_multiplier = {1,1}; //might gotta change these values I just put something here

    private double shoot_theta,turret_theta, max_velocity;
    private double curr_x,curr_y,shoot_distance;

    public Deposit(Robot robot) {
        this.robot = robot;
        shooter = new PriorityMotor(shooters,"turret",4.0, 5.0,shooter_multiplier, null);
        cloth = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "cloth")}, "cloth", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        hood = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "hood")}, "hood", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        turret = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "turret1"),robot.hardwareMap.get(Servo.class,"turret2")}, "turret", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false, false}, 2, 5);

        robot.hardwareQueue.addDevices(shooter,cloth,hood,turret);
    }

    public enum DepositStates {
        IDLE,
        TRANSFER,
        TRANFER_READY,
        SHOOT,

    }

    public DepositStates depositStates = DepositStates.IDLE;

    public void update(){
        switch (depositStates){
            case IDLE:
                break;
            case TRANSFER:

            case TRANFER_READY:


        }
    }

    public void setTurretPos(double target_angle){turret.setTargetAngle(target_angle);}
    public void setHoodPos(double target_angle){hood.setTargetAngle(target_angle);}
    public void setClothPos(double target_angle){cloth.setTargetAngle(target_angle);}
    public void setShooterPower(double power){shooter.setTargetPower(power);}

    public void shootBall(double target_x,double target_y, double target_z){
        curr_x = globals.ROBOT_POSITION.x;
        curr_y = globals.ROBOT_POSITION.y;
        shoot_distance = Math.sqrt((target_x-curr_x)*(target_x-curr_x) + (target_y-curr_y)*(target_y-curr_y));
        shoot_theta = Math.atan(max_velocity - 9.8*shoot_distance);
        // pull the turret heading from the limelight
    }
}
