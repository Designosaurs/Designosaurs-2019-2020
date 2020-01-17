package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Jan Skystone Auto", group = "!Auto")
public class SkystoneAutoJan extends LinearOpMode {
    ImuSubClass imu = new ImuSubClass();
    Hardware robot = new Hardware();
    ElapsedTime time = new ElapsedTime();

    int targetStoneNumber = 0; // numbered from the inside (toward bridge), starting with 1.


    boolean enabableStops = true; // Set to true to stop between steps for debugging.

    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critical parts.
    private void waitForYellow() {
        if (!enabableStops) return;
        while (true) {
            robot.stopDrive();
            if (gamepad1.y) break;
            //updateSensors();
        }
        while (true) {
            robot.stopDrive();
            if (!gamepad1.y) break;
            // updateSensors();
        }
    }

    // Call this when facing the stone but a few inchest away.
    // Return true if this is the target stone.
    // Will leave the bot right next to the stone.
    boolean checkForTarget() {
        robot.driveToProx(8.0, 6.0, this);
        waitForYellow();

        return !robot.seesYellow(this);
    }

    void grabStone() {
        // Creep until get into the Yellow.
        robot.driveToColorEdge(Hardware.Direction.LEFT, 6.0, true, this);
        waitForYellow();

        // Strafe to where we came from to get the grabber centered.
        robot.moveRampToPosition("right", .3, 9.5, robot, this, time);
        waitForYellow();

        // Backward (toward stone)  to be ready to grab that stone.
        robot.moveRampToPosition("backward", .4, 1.0, robot, this, time);
        waitForYellow();

        // Deploy manipulator
        robot.deployLeftAutoManipulator();
        waitForYellow();

        // Ease the stone out
        robot.moveRampToPosition("forward", .2, 8, robot, this, time);
        waitForYellow();
    }

    ////////////////////////////////////  THE RUN PROGRAM ///////////////////////////////////////////////
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

        waitForStart();
        imu.ReadIMU();


        // Robot starting position is with side to wall, and back facing the bridge.
        // Wheels on the inside seam of the second tile from corner.

        // Strafe away from wall.
        robot.moveRampToPosition("left", .4, 13.5, robot, this, time);
        waitForYellow();

        // Go toward bridge a tiny bit to be right in front of stone #1.
        robot.moveRampToPosition("backward", .4, 3, robot, this, time);
        waitForYellow();

        // Rotate the robot so the back (sensor / manipulator) side faces stones.
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnAndCorrect(-90, robot, this);
        waitForYellow();

        // Go (backward) to just a few inches fron stone #1.
        robot.moveRampToPosition("backward", .3, 11, robot, this, time);
        waitForYellow();


        if (checkForTarget()) {
            // It is stone #1.  Grab it.
            targetStoneNumber = 1;
            grabStone();
        }

        // Get some clearance
        // robot.moveRampToPosition( "forward", .4,10,robot,this,time);


        //robot.moveRampToPosition(Hardware.Direction.BACKWARD, .4, 6, robot, this, time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftAutoManipulator.setPosition(0);
        //robot.wait(1,this,time);

        //robot.moveRampToPosition(Hardware.Direction.LEFT, .6, 50,robot,this,time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lockOn();
        //robot.moveRampToPosition(Hardware.Direction.RIGHT,.6,50,robot,this,time);
        //robot.leftAutoManipulator.setPosition(0);
        //robot.wait(1,this,time);
        //robot.moveRampToPosition(Hardware.Direction.LEFT,.6,10,robot,this,time);

    }


}
