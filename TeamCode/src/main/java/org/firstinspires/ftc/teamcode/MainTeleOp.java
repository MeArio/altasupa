
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleOp Relic Recovery", group="TeleOp")
public class MainTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    boolean beforeX = false;
    boolean drop = false;
    boolean seenX = false;
    boolean seenA = false;
    boolean liftB = false;
    boolean seenB = false;
    boolean collectB = false;
    boolean precision = false;
    boolean seenBumper = false;
    boolean seenY = false;
    boolean push = false;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // robot movement
            // angle of left analog stick

            // robot rotation
            // angle of right analog stick

            robot.motorPowers[0] = 0;
            robot.motorPowers[1] = 0;
            robot.motorPowers[2] = 0;
            robot.motorPowers[3] = 0;

            if(gamepad1.right_bumper && seenBumper) {
                precision = !precision;
                seenBumper = false;
            } else if(!gamepad1.right_bumper)
                seenBumper = true;

            if(gamepad1.a && seenA) {
                liftB = !liftB;
                seenA = false;
            } else if(!gamepad1.a)
                seenA = true;
            robot.liftCubes(liftB);

            if(gamepad1.y && seenY) {
                push = !push;
                seenY = false;
            } else if(!gamepad1.y)
                seenY = true;

            if(gamepad1.x && seenX) {
                drop = !drop;
                seenX = false;
            } else if(!gamepad1.x)
                seenX = true;
            robot.drop(drop, liftB);

            if(liftB || drop)
                push = false;

            robot.pushCubes(push);

            //telemetry.addData("Drop ", robot.lift.getCurrentPosition());
            //telemetry.update();

            //double angle_left = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI/2;
            double angle_right = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + Math.PI/2;

            if(gamepad1.right_stick_button)
                robot.targetAngle = (int) Math.toDegrees(angle_right);

            robot.rotateUpdate();

            double angle_left = -1;

            if(gamepad1.dpad_right)
                angle_left = Math.PI/2;
            if(gamepad1.dpad_up)
                angle_left = 0;
            if(gamepad1.dpad_left)
                angle_left = Math.PI*3.0/2.0;
            if(gamepad1.dpad_down)
                angle_left = Math.PI;

            robot.moveUpdate(angle_left, (angle_left == -1) ? ( 0.0 ) :  ( (precision ? gamepad1.left_trigger / 2.5: gamepad1.left_trigger) ) );
            robot.composePresesmeker();

            if(gamepad1.b && seenB) {
                collectB = !collectB;
                seenB = false;
                if(collectB)
                    robot.elapsed = System.currentTimeMillis();
            } else if(!gamepad1.b)
                seenB = true;
            if(gamepad1.right_trigger >= 0.6) {
                collectB = false;
                robot.collect(true, -0.33, false);
            } else
                robot.collect(collectB, 0.33, true);

            beforeX = gamepad1.x;

        }

    }
}
