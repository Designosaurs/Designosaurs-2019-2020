package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Test Sensors", group = "!Auto")
public class TestSensors extends LinearOpMode {
    final double SCALE_FACTOR = 255;
    ImuSubClass imu = new ImuSubClass();
    Hardware robot = new Hardware();
    ElapsedTime time = new ElapsedTime();
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float[] hsvValues = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float[] values = hsvValues;
    double imuValue = imu.getHeading();

    @Override
    public void runOpMode() {
        time.reset();
        robot.init2(hardwareMap, 0, 0, 0);

        waitForStart();
        while (opModeIsActive()) {

            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            double RGtoBRatio;
            RGtoBRatio = ((double) robot.sensorColor.red() + (double) robot.sensorColor.green()) / (double) robot.sensorColor.blue();

            // send the info back to driver station using telemetry function.
            telemetry.addData("Green", "Exit loop");
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("RGtoBRaion", RGtoBRatio);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Heading", imuValue);

            telemetry.addData("Prox (cm)",
                    String.format(Locale.US, "%.01f", robot.sensorDistance.getDistance(DistanceUnit.CM)));

            telemetry.addData("Range Front (In)", "%.01f", robot.sensorRangeFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("Range Right (In)", "%.01f", robot.sensorRangeRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("Range Left (In)", "%.01f", robot.sensorRangeLeft.getDistance(DistanceUnit.INCH));

            // telemetry.addData("Dist",robot.getDistance());
            telemetry.update();
            if (gamepad1.a) break;
            //updateSensors();
        }
        while (true) {
            robot.stopDrive();
            if (!gamepad1.a) break;
            // updateSensors();
        }
    }
}
