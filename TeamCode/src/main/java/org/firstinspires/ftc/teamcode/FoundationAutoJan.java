package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FoundationAutoJan", group = "!Auto")
public class FoundationAutoJan extends LinearOpMode {
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
        if (teamColorBlue == true) {
            // Make sure gripper is in up position
            robot.foundationGripper.setPosition(0.67);

            // Move away from the wall to avoid knocking off the capstone
            robot.moveRampToPosition("right", .4, 4, robot, this, time);

            // Turn toward and position in front of the foundation
            //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            imu.turnAndCorrect(90, robot, this);
            robot.moveRampToPosition("right", .4, 16, robot, this, time);
            sleep((100));
            robot.moveRampToPosition("backward", .4, 20, robot, this, time);
            imu.correctHeading(90, robot, this);
            robot.moveRampToPosition("backward", .1, 10, robot, this, time);

            // Grab foundation and move it into the building site
            robot.foundationGripper.setPosition(0.9);
            sleep(100);
            robot.moveRampToPosition("forward", .4, 23, robot, this, time);
            robot.moveRampToPosition("forward", .1, 11, robot, this, time);

            // Let go and park under the skybridge
            robot.foundationGripper.setPosition(0.67);
            sleep(200);
            robot.moveRampToPosition("left", .4, 23, robot, this, time);
            sleep(5000);
            imu.turnAndCorrect(0, robot, this);
            robot.moveRampToPosition("backward", .4, 22, robot, this, time);
        } else if (teamColorBlue == false) {
            /////////////////////////////////////////////////  RED SIDE ///////////////////////////////////////////////////////////////////
            // Make sure gripper is in up position
            robot.foundationGripper.setPosition(0.67);

            // Move away from the wall to avoid knocking off the capstone
            robot.moveRampToPosition("left", .4, 4, robot, this, time);

            // Turn toward and position in front of the foundation
            //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            imu.turnAndCorrect(-90, robot, this);
            robot.moveRampToPosition("left", .4, 16, robot, this, time);
            //imu.turnSimp(-90,robot,this);
            sleep(100);
            robot.moveRampToPosition("backward", .4, 22, robot, this, time);
            imu.correctHeading(-90, robot, this);
            robot.moveRampToPosition("backward", .1, 10, robot, this, time);

            // Grab foundation and move it into the building site
            robot.foundationGripper.setPosition(0.9);
            sleep(100);
            robot.moveRampToPosition("forward", .4, 23, robot, this, time);
            robot.moveRampToPosition("forward", .1, 11, robot, this, time);

            // Let go and park under the skybridge
            robot.foundationGripper.setPosition(0.67);
            sleep(200);
            robot.moveRampToPosition("right", .4, 23, robot, this, time);
            sleep(1000);
            imu.turnAndCorrect(0, robot, this);
            robot.moveRampToPosition("backward", .4, 22, robot, this, time);
        }
        //lockOn(false,false, .4);

        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.BACKWARD, .4, 6, robot, this, time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.mainGripperLeft.setPosition(0);
        //robot.wait(1,this,time);

        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.LEFT, .6, 50,robot,this,time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lockOn();
        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.RIGHT,.6,50,robot,this,time);
        //robot.mainGripperLeft.setPosition(0);
        //robot.wait(1,this,time);
        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.LEFT,.6,10,robot,this,time);

    }


}
