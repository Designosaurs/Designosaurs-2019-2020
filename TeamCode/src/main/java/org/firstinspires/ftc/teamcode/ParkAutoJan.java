package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Park Auto Jan", group = "!Auto")
public class ParkAutoJan extends LinearOpMode {
    final double SCALE_FACTOR = 255;
    ImuSubClass imu = new ImuSubClass();
    Hardware robot = new Hardware();
    ElapsedTime time = new ElapsedTime();
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float[] hsvValues = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float[] values = hsvValues;
    boolean enabableStops = true; // Set to true to stop between steps for debugging.

    boolean teamColorBlue = true;

    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critical parts.
    private void waitForGreen() {
        if (!enabableStops) return;
        while (true) {
            robot.stopDrive();
            if (gamepad1.a) break;
            //updateSensors();
        }
        while (true) {
            robot.stopDrive();
            if (!gamepad1.a) break;
            // updateSensors();
        }
    }


    @Override
    public void runOpMode() {
        time.reset();
        telemetry.addData("selected", "Blue");
        telemetry.update();

        telemetry.addData("init", "Imu");
        imu.init(hardwareMap);
        telemetry.addData("init", "Robot");
        telemetry.update();
        robot.init2(hardwareMap, 0, 0, 0);
        telemetry.addData("init", "done");
        telemetry.update();


        // Team color selection
        while (!isStarted()) {
            if (gamepad1.x) {
                teamColorBlue = true;
            }
            if (gamepad1.b) {
                teamColorBlue = false;
            }
            if (teamColorBlue) {
                telemetry.addData("Team Color>", "BLUE BLUE BLUE");
            } else {
                telemetry.addData("Team Color>", "RED RED RED");
            }
            telemetry.update();
        }

        waitForStart();
        imu.ReadIMU();


        //robot.moveRampToPosition("right", .4,13.5,robot,this,time);

        /////////////////////////////////////////////////  BLUE SIDE ///////////////////////////////////////////////////////////////////

        robot.moveRampToPosition("backward", .4, 28, robot, this, time);


    }


}
