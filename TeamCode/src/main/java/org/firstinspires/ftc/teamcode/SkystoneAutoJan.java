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
    boolean enableStops = false; // Set to true to stop between steps for debugging.

    // constant for size of playing element
    double stoneLength = 7.0;

    // Stuff that changes with BLUE and RED
    // These are set up for if we are on the blue side.
    // They refer to the directions with respect to the robot for after it has made its 90
    // degree turn, so the back is facing the skystones.
    String inside = "right";
    String outside = "left";
    double turnToFaceStones = -90;


    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critical parts.
    private void waitForYellow() {
        if (!enableStops) return;
        while (true) {
            robot.stopDrive();
            if (gamepad1.y) break;
            //updateSensors();
            sleep(250);
        }
        while (true) {
            robot.stopDrive();
            if (!gamepad1.y) break;
            // updateSensors();
            sleep(250);
        }
    }

    // Call this when facing the stone but a few inchest away.
    // Return true if this is the target stone.
    // Will leave the bot right next to the stone.
    boolean checkForTarget() {
        robot.driveToProx(6.0, 6.0, this);
        waitForYellow();

        return !robot.seesYellow(this);
    }

    void grabStone() {
        // Any slght angle misalignment will mean the sensor will get closer/further as it moves, so
        // make sure it is aimedd right.
        imu.correctHeading( turnToFaceStones, robot, this );

        // Creep until get into the Yellow.
        robot.driveToColorInsideEdge(Hardware.Direction.RIGHT, 8.0, this);

        waitForYellow();

        // Strafe to where we came from to get the grabber centered.
        robot.moveRampToPosition("right", .2, 3, robot, this, time);
        //waitForYellow();

        // Backward (toward stone)  to be ready to grab that stone.
        robot.moveRampToPosition("backward", .4, .7, robot, this, time);
        //waitForYellow();

        // Deploy manipulator
        robot.deployRightAutoManipulator();
        sleep(800);
        //waitForYellow();

        // Ease the stone out
        robot.moveRampToPosition("forward", .6, 14, robot, this, time);
        waitForYellow();
    }

    // FIND THE TARGET STONE
    void seekForStone() {
        double distanceToGo;

        // See if it is stone number 1.
        if (checkForTarget()) {
            // It is stone #1.  Grab it.
            targetStoneNumber = 1;
            grabStone();
            waitForYellow();
        }
        // If target was not stone 1, see if it is stone 2.
        if (targetStoneNumber == 0){
            waitForYellow();
            // Back a tiny bit
            robot.moveRampToPosition("forward", .4, 1, robot, this, time);
            // Move to outside
            robot.moveRampToPosition("left", .4, stoneLength, robot, this, time);
            waitForYellow();

            if (checkForTarget()) {
                // It is stone #2.  Grab it.
                targetStoneNumber = 2;
                grabStone();
            }
        }
        // If we have not found the target yet, see if it is stone #3.
        if (targetStoneNumber == 0){
            waitForYellow();
            // Back a tiny bit
            robot.moveRampToPosition("forward", .3, 1, robot, this, time);
            // Move to outside
            robot.moveRampToPosition("left", .3, stoneLength, robot, this, time);
            waitForYellow();

            if (checkForTarget()) {
                // It is stone #3.  Grab it.
                targetStoneNumber = 3;
                grabStone();
            } else {
                // We cound not find the stone!  Snap!
                // Our best bet is to park under the bridge.
                // Things appear to have gone awry and that is the least ambitious move.
                // You are two stone lengths further outside than stone 1.
                distanceToGo = 19 + 2.0 * stoneLength;
                robot.moveRampToPosition("right", .4, distanceToGo, robot, this, time);
                waitForYellow();
            }
        }
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

        // POSITION IN FRONT OF FIRST STONE
        // Strafe away from wall.
        robot.moveRampToPosition("left", .8, 13.5, robot, this, time);
        //waitForYellow();

        // Go toward bridge a tiny bit to be right in front of stone #1.
        robot.moveRampToPosition("backward", .4, 4.5, robot, this, time);
        //waitForYellow();

        // Rotate the robot so the back (sensor / manipulator) side faces stones.
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnAndCorrect(turnToFaceStones, robot, this);
        //waitForYellow();

        // Go toward the stone (backward) to just a few inches fron stone #1.
        robot.moveRampToPosition("backward", .3, 12, robot, this, time);
        //waitForYellow();

        seekForStone();

        // SHUTTLE IT TO THE OTHER SIDE
        // Go under the bridge.
        double distanceToGo = 30 + (stoneLength * ( (double) targetStoneNumber - 1.0));
        robot.moveRampToPosition("right", .7, distanceToGo, robot, this, time);
        waitForYellow();

        // Drop the stone and park under the bridge.
        robot.resetLeftAutoManipulator();
        // give it time to let go of the block:
        sleep(500);

        // park under the bridge, we don't have time:
        if (targetStoneNumber == 3) {
            robot.moveRampToPosition("left", .5, 10, robot, this, time);
        }

        //imu.correctHeading(turnToFaceStones, robot, this);
        //distanceToGo = 36 + 2.0 * stoneLength * ( (double) targetStoneNumber - 1.0)
        distanceToGo = 9;
        robot.moveRampToPosition("left", .8, distanceToGo + 44, robot, this, time);
        imu.correctHeading(turnToFaceStones, robot, this);
        waitForYellow();

        // approach the blocks again.
        robot.moveRampToPosition("backward", .5, 8, robot, this, time);
        targetStoneNumber = 0;
        seekForStone();
        distanceToGo = 40;
        robot.moveRampToPosition("right", .8, distanceToGo, robot, this, time);
    }
}
