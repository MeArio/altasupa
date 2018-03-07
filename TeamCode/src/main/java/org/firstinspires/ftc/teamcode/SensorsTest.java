
package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SensorsTest Relic Recovery", group="TeleOp")
public class SensorsTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    @Override
    public void runOpMode() {

        /////////////
        //robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ModernRoboticsI2cCompassSensor compass = (ModernRoboticsI2cCompassSensor) hardwareMap.compassSensor.get("compass");
        //AccelerationSensor accel = hardwareMap.accelerationSensor.get("compass");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Compass", ": " + compass.getAcceleration() + " + " + compass.getDirection());
            telemetry.update();
        }
    }
}
