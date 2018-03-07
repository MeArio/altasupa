package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
/**
 * Created by Tesserakt on 2/5/2018.
 */

public class Robot {

    //private static final DcMotor base;
    private static final HashMap<String, DcMotor> motors = new HashMap<>();
    private static final HashMap<String, Servo> servos = new HashMap<>();
    private static final HashMap<String, ModernRoboticsI2cGyro> gyros = new HashMap<>();
    private static final HashMap<String, DcMotor> collectors = new HashMap<>();
    Servo pusher;
    DcMotor lift;
    int liftTarget = (int) ( ( 8.0 * 1120.0 * 2.0 ) / 3.0 / 60.0 * 3.7 ); // 2 rotations
    int liftDeviation = 10;
    int currentAngle = 0;
    int targetAngle = 0;
    double motorPowers[] = new double[4];
    HardwareMap map;

    double speedFactor = 0.5;
    double rotateFactor = 0.125;

    double servoDifference = 32.0/255.0;
    double servoInitial = 252.0/255.0;

    double rotatePower = 0.23;

    double elapsed = 0;

    private double p, i, d;
    // 232  252 servos

    public Robot(HardwareMap map) {

        this.map = map;
        motors.put("dc_left_up", map.get(DcMotor.class, "dc_left_up"));
        motors.put("dc_left_down", map.get(DcMotor.class, "dc_left_down"));
        motors.put("dc_right_up", map.get(DcMotor.class, "dc_right_up"));
        motors.put("dc_right_down", map.get(DcMotor.class, "dc_right_down"));

        collectors.put("collector1", map.get(DcMotor.class, "collector1"));
        collectors.put("collector2", map.get(DcMotor.class, "collector2"));

        servos.put("drop1", map.get(Servo.class, "drop1"));
        servos.put("drop2", map.get(Servo.class, "drop2"));

        servos.get("drop1").setDirection(Servo.Direction.REVERSE);
        servos.get("drop2").setDirection(Servo.Direction.FORWARD);

        gyros.put("gyro", map.get(ModernRoboticsI2cGyro.class, "gyro"));
        gyros.get("gyro").resetZAxisIntegrator();
        gyros.get("gyro").calibrate();

        pusher = map.get(Servo.class, "pusher");

        while (gyros.get("gyro").isCalibrating())  {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        lift = map.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors.get("dc_left_up").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("dc_left_down").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("dc_right_up").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("dc_right_down").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public Robot move(double angle) {

        motors.get("dc_left_up").setPower(Math.sin(angle - Math.PI / 4));
        motors.get("dc_left_down").setPower(Math.cos(angle - Math.PI / 4));
        motors.get("dc_right_up").setPower(-Math.cos(angle - Math.PI / 4));
        motors.get("dc_right_down").setPower(-Math.sin(angle - Math.PI / 4));

        return this;

    }

    public Robot moveUpdate(double angle, double power) {

        angle = (Math.toDegrees(angle) + 720) % 360;

        int upd = 0;

        if(angle <= 45 || angle >= 315) {
            upd = 1;
            motorPowers[0] += speedFactor * power;
            motorPowers[1] += speedFactor * power;
            motorPowers[2] += speedFactor * power;
            motorPowers[3] += speedFactor * power;
        }

        if(angle >= 45 && angle <= 135) {
            upd = 2;
            motorPowers[0] += -speedFactor * power;
            motorPowers[1] += speedFactor * power;
            motorPowers[2] += speedFactor * power;
            motorPowers[3] += -speedFactor * power;
        }

        if(angle >= 135 && angle <= 225) {
            upd = 3;
            motorPowers[0] += -speedFactor * power;
            motorPowers[1] += -speedFactor * power;
            motorPowers[2] += -speedFactor * power;
            motorPowers[3] += -speedFactor * power;
        }

        if(angle >= 225 && angle <= 315) {
            upd = 4;
            motorPowers[0] += speedFactor * power;
            motorPowers[1] += -speedFactor * power;
            motorPowers[2] += -speedFactor * power;
            motorPowers[3] += speedFactor * power;
        }

        return this;

    }

    public Robot rotateUpdate() {

        currentAngle = -gyros.get("gyro").getHeading();

        double angleDif = targetAngle - currentAngle;
        angleDif = angleDif+720;
        while(angleDif > 360)
            angleDif -= 360;

        if (angleDif > 0 && angleDif < 180) {
            double forceHeiHei = Math.pow(angleDif / 180.0, rotatePower) * rotateFactor;
            motorPowers[0] += forceHeiHei;
            motorPowers[1] += forceHeiHei;
            motorPowers[2] -= forceHeiHei;
            motorPowers[3] -= forceHeiHei;
        } else {
            double forceHeiHei = Math.pow((360 - angleDif) / 180.0, rotatePower) * rotateFactor;
            motorPowers[0] -= forceHeiHei;
            motorPowers[1] -= forceHeiHei;
            motorPowers[2] += forceHeiHei;
            motorPowers[3] += forceHeiHei;
        }

        //TestTeleOp.teleme.addData("Motors", angleDif + " + " + motorPowers[0] + " + " + motorPowers[1] + " + " + motorPowers[2] + " + " + motorPowers[3]);
        //TestTeleOp.teleme.update();

        return this;

    }

    public Robot composePresesmeker() {

        motors.get("dc_left_up").setPower(motorPowers[0]);
        motors.get("dc_left_down").setPower(motorPowers[1]);
        motors.get("dc_right_up").setPower(motorPowers[2]);
        motors.get("dc_right_down").setPower(motorPowers[3]);

        return this;

    }

    public Robot liftCubes(boolean up) {

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(up ? liftTarget : 0);

        if((lift.getCurrentPosition() >= -liftDeviation && lift.getCurrentPosition() <= liftDeviation && !up) ||
                (lift.getCurrentPosition() >= liftTarget-liftDeviation && lift.getCurrentPosition() <= liftTarget+liftDeviation && up))
            lift.setPower(0);
        else
            lift.setPower(0.5);

        return this;

    }

    public Robot rotate(double magnitude) {

    /*       if (left == true) {
            motors.get("dc_left_up").setPower(-1);
            motors.get("dc_left_down").setPower(-1);
            motors.get("dc_right_up").setPower(-1);
            motors.get("dc_right_down").setPower(-1);
        }
        else {
            motors.get("dc_left_up").setPower(1);
            motors.get("dc_left_down").setPower(1);
            motors.get("dc_right_up").setPower(1);
            motors.get("dc_right_down").setPower(1);
        }*/
        motors.get("dc_left_up").setPower(-magnitude);
        motors.get("dc_left_down").setPower(-magnitude);
        motors.get("dc_right_up").setPower(-magnitude);
        motors.get("dc_right_down").setPower(-magnitude);
        return this;

    }

    public Robot stop(boolean gradual) {

        // Float if gradual is true and brake if gradual is false
        motors.get("dc_left_up").setZeroPowerBehavior(gradual ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("dc_left_down").setZeroPowerBehavior(gradual ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("dc_right_up").setZeroPowerBehavior(gradual ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get("dc_right_down").setZeroPowerBehavior(gradual ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

        motors.get("dc_left_up").setPower(0);
        motors.get("dc_left_down").setPower(0);
        motors.get("dc_right_up").setPower(0);
        motors.get("dc_right_down").setPower(0);

        return this;

    }

    public Robot drop(boolean drop, boolean lifting) {

        //servos.get("drop1").setPosition(drop ? -servoInitial+servoDropDifference+servoDifference + fix : -servoInitial+servoDifference + fix);
        //servos.get("drop2").setPosition(drop ? servoInitial+servoDropDifference - fix : servoInitial - fix);

        if(drop) {

            servos.get("drop1").setPosition(1 - (servoInitial - 0.84));
            servos.get("drop2").setPosition(servoInitial - servoDifference - 0.84);

        } else {

            if(lifting) {

                servos.get("drop1").setPosition(1 - (servoInitial - 0.25));
                servos.get("drop2").setPosition(servoInitial - servoDifference - 0.25);

            } else {

                servos.get("drop1").setPosition(1-servoInitial);
                servos.get("drop2").setPosition(servoInitial - servoDifference);

            }

        }



        return this;

    }

    public Robot pushCubes(boolean push) {

        pusher.setPosition(push ? 0: 1);

        return this;

    }

    public Robot collect(boolean coll, double power, boolean magic) {

        /*if(magic) {

            double time = System.currentTimeMillis() - elapsed;
            time = time % 1000;

            if(time < 700) {
                power *= 1;
            } else {
                power *= -1;
            }

        }*/

        collectors.get("collector1").setPower((coll ? 1 : 0) * power);
        collectors.get("collector2").setPower((coll ? 1 : 0) * power);

        return this;

    }

}
