package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "State Scan Auto", group = "!Auto")
public class SkystoneAutoScan extends LinearOpMode {
    ImuSubClass imu = new ImuSubClass();
    Hardware robot = new Hardware();
    ElapsedTime time = new ElapsedTime();

    int targetStoneNumber = 0; // numbered from the inside (toward bridge), starting with 1.
    boolean enableStops = false; // Set to true to stop between steps for debugging.
    boolean teamColorBlue = true;
    boolean getSecondStone = true;
    boolean secondTimeAround = false;
    boolean truncateAfterOne = true; // Determines whether or not we go for a second stone.

    // constant for size of playing element
    double stoneLength = 7.0;
    double distanceToGo = 0;

    double skystoneEndoderDifference = 0;

    // Stuff that changes with BLUE and RED
    // These are set up for if we are on the blue side.
    // They refer to the directions with respect to the robot for after it has made its 90
    // degree turn, so the back is facing the skystones.
    String inside = "right";
    String outside = "left";
    double turnToFaceStones = -90;
    double slightlyIn = -85;


    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critical parts.
    private void waitForYellow() {
        if (!enableStops) return;
        while (true && opModeIsActive()) {
            robot.stopDrive();
            if (gamepad1.y) break;
            //updateSensors();
            sleep(250);
        }
        while (true && opModeIsActive()) {
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
        if (teamColorBlue) {
            //robot.driveToRange(2.0, "backward", 0.1, 3, this);
        } else {
            // robot.driveToRange(2.0, "backward", 0.1, 3, this);
        }
        waitForYellow();

        return !robot.seesYellow(this);
    }

    boolean checkForTargetNoMove() {
        waitForYellow();

        return !robot.seesYellow(this);
    }

    void grabStone() {
        // Any slght angle misalignment will mean the sensor will get closer/further as it moves, so
        // make sure it is aimedd right.
        imu.correctHeading(turnToFaceStones, robot, this);
        robot.brake();

        // Creep until get into the Yellow.
        if (teamColorBlue) {
            robot.driveToColorInsideEdge(Hardware.Direction.RIGHT, 8.0, this);
        } else {
            robot.driveToColorInsideEdge(Hardware.Direction.LEFT, 8.0, this);
        }
        robot.brake();
        //imu.correctHeading(turnToFaceStones, robot, this);

        waitForYellow();

        // At this point, we know where we are relative to the stones
        // Strafe to where we came from to get the grabber centered.
        if (targetStoneNumber == 1) {
            // It's really important that the distance the robot moves is different for red and blue because the color sensor is not in the exact center of the robot.
            if (teamColorBlue) {
                robot.moveRampToPosition(inside, .2, 3.0, robot, this, time);
            } else {
                robot.moveRampToPosition(inside, 0.2, 1.5, robot, this, time);
            }
        } else {
            if (teamColorBlue) {
                robot.moveRampToPosition(inside, .2, 3.5, robot, this, time);
            } else {
                robot.moveRampToPosition(inside, .2, 1.5, robot, this, time);
            }
        }
        //waitForYellow();

        // Backward (toward stone)  to be ready to grab that stone.
        robot.moveRampToPosition("backward", .4, 1.2, robot, this, time);
        //waitForYellow();

        // Deploy manipulator
        if (teamColorBlue) {
            robot.deployLeftAutoManipulator();
            sleep(300);
        } else {
            robot.deployRightAutoManipulator();
            sleep(300);
        }


        //waitForYellow();

        //imu.correctHeading(turnToFaceStones, robot, this);
        // Ease the stone out
        if (teamColorBlue) {
            if (secondTimeAround) {
                robot.moveRampToPosition("forward", .6, 25, robot, this, time);
            } else {
                robot.moveRampToPosition("forward", .7, 18, robot, this, time);
            }
        } else {
            //robot.moveRampToPosition("forward", .6, 16, robot, this, time);
            if (secondTimeAround) {
                robot.moveRampToPosition("forward", .6, 23, robot, this, time);
            } else {
                robot.moveRampToPosition("forward", .7, 18, robot, this, time);
            }
        }
        secondTimeAround = true;
        waitForYellow();
    }

    // Finds the right stone by driving until the robot sees it
    void driveByGrab() {
        if (checkForTarget()) {
            // It is stone #1.  Grab it.
            targetStoneNumber = 1;
            grabStone();
            telemetry.addData("It is", "This One!!!!");
            waitForYellow();
        }
        //sleep(1000);

        if (targetStoneNumber == 0) {
            if (teamColorBlue) {
                robot.driveToSkystoneEdge(Hardware.Direction.LEFT, 750, this);
            } else {
                robot.driveToSkystoneEdge(Hardware.Direction.RIGHT, 750, this);
            }
            telemetry.addData("Second Stone Difference", getSecondStonePosition());
            telemetry.addData("Encoder Difference", skystoneEndoderDifference);
            telemetry.update();
            targetStoneNumber = getSecondStonePosition();
            waitForYellow();
            grabStone();
        }
        // If target was not stone 1, see if it is stone 2.
        /*
        if (targetStoneNumber == 0){
            waitForYellow();
            // Back a tiny bit
            robot.moveRampToPosition("forward", .4, 1, robot, this, time);
            imu.correctHeading(turnToFaceStones, robot, this);            // Move to outside
            robot.moveRampToPosition(outside, .4, stoneLength, robot, this, time);
            waitForYellow();

            if (checkForTarget()) {
                // It is stone #2.  Grab it.
                targetStoneNumber = 2;
                grabStone();
            }
        }

         */
    }

    public int getSecondStonePosition() {
        skystoneEndoderDifference = Math.abs(robot.skystoneEncoderInitial - robot.skystoneEncoderEnd);
        if (teamColorBlue) {
            if (skystoneEndoderDifference >= 100 && skystoneEndoderDifference <= 340) {
                return 2;
            } else {
                return 3;
            }
        } else {
            if (skystoneEndoderDifference >= 150 && skystoneEndoderDifference <= 250) {
                return 2;
            } else {
                return 3;
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

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                teamColorBlue = true;
                inside = "right";
                outside = "left";
                turnToFaceStones = -90;
                slightlyIn = -85;
            }
            if (gamepad1.b) {
                teamColorBlue = false;
                inside = "left";
                outside = "right";
                turnToFaceStones = 90;
                slightlyIn = 80;
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
        // Robot starting position is with side to wall, and back facing the bridge.
        // Wheels on the inside seam of the second tile from corner.

        // POSITION IN FRONT OF FIRST STONE
        // Strafe away from wall.
        robot.moveRampToPosition(outside, .9, 20.5, robot, this, time);
        //waitForYellow();

        // Go toward bridge a tiny bit to be right in front of stone #1.
        if (teamColorBlue) {
            robot.moveRampToPosition("backward", .5, 2.0, robot, this, time);
        } else {
            robot.moveRampToPosition("backward", .5, 2.0, robot, this, time);
        }
        //waitForYe                                                                             llow();

        // Rotate the robot so the back (sensor / manipulator) side faces stones.
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnAndCorrect(turnToFaceStones, robot, this);
        robot.brake();
        sleep(200);
        imu.correctHeading(turnToFaceStones, robot, this);
        robot.brake();
        //waitForYellow();

        // Go toward the stone (backward) to just a few inches from stone #1. If it fails to see the stones, we'll park under the bridge
        //robot.driveToRange(6, "backward", 0.2, 1, this);
        // robot.brake();
        if (!robot.driveToRange(2, "backward", 0.10, 3, this)) {
            robot.moveRampToPosition("forward", 0.3, 18, robot, this, time);
            robot.moveRampToPosition(inside, 0.3, 26, robot, this, time);
            sleep(100000);
        }
        robot.brake();
        robot.moveRampToPosition("backward", 0.05, 1.0, robot, this, time);
        robot.brake();


        imu.correctHeading(turnToFaceStones, robot, this);
        robot.brake();

        driveByGrab();
        imu.correctHeading(slightlyIn, robot, this);
        robot.brake();

        waitForYellow();

        //sleep(100000000);

        // SHUTTLE IT TO THE OTHER SIDE
        // Go under the bridge.
        if (teamColorBlue) {
            // If we're BLUE:
            if (targetStoneNumber == 1) {
                distanceToGo = 34;
            } else if (targetStoneNumber == 2) {
                distanceToGo = 41;
            } else {
                distanceToGo = 49;
            }
        } else {
            // If we're RED:
            if (targetStoneNumber == 1) {
                distanceToGo = 34;
            } else if (targetStoneNumber == 2) {
                distanceToGo = 38;
            } else {
                distanceToGo = 46;
            }
        }
        robot.moveRampToPosition(inside, 1.0, distanceToGo, robot, this, time);
        waitForYellow();

        // Let go of the stone
        if (teamColorBlue) {
            robot.resetLeftAutoManipulator();
        } else {
            robot.resetRightAutoManipulator();
        }
        // give it time to let go of the block:
        sleep(200);
        robot.moveRampToPosition("forward", 0.5, 1, robot, this, time);
        imu.correctHeading(turnToFaceStones, robot, this);
        robot.brake();
        if (truncateAfterOne) {
            getSecondStone = false;
        }


        if (truncateAfterOne) {
            if (teamColorBlue) {
                robot.moveRampToPosition(outside, .5, 19, robot, this, time);
            } else {
                robot.moveRampToPosition(outside, .5, 19, robot, this, time);
            }
        }

        if (getSecondStone) {
            double distanceFromWall = 0;
            double distanceToShuttle = 0;
            if (targetStoneNumber == 1) {
                robot.moveRampToPosition(outside, 1, 50, robot, this, time);
                distanceFromWall = 20;
                distanceToShuttle = 50;
            }
            if (targetStoneNumber == 2) {
                robot.moveRampToPosition(outside, 1, 56, robot, this, time);
                distanceFromWall = 12;
                distanceToShuttle = 58;
            } else if (targetStoneNumber == 3) {
                robot.moveRampToPosition(outside, 1, 62, robot, this, time);
                distanceFromWall = 6;
                distanceToShuttle = 66;
            }
            // Get to the right position to grab the second stone
            robot.driveToRange(distanceFromWall, outside, 0.2, 5, this);
            robot.brake();
            imu.correctHeading(turnToFaceStones, robot, this);
            robot.brake();
            //sleep (5000);
            //robot.moveRampToPosition("backward", 0.3, 4, robot, this, time);

            // Approach the stone
            robot.driveToRange(2, "backward", 0.3, 5, this);
            robot.moveRampToPosition("backward", 0.1, 1, robot, this, time);
            if (teamColorBlue) {
                robot.deployLeftAutoManipulator();
                sleep(300);
            } else {
                robot.deployRightAutoManipulator();
                sleep(300);
            }
            // Pull it out.
            robot.moveRampToPosition("forward", 0.5, 14, robot, this, time);
            imu.correctHeading(slightlyIn, robot, this);
            // Shuttle it under the bridge
            robot.moveRampToPosition(inside, 1, distanceToShuttle, robot, this, time);
            if (teamColorBlue) {
                robot.resetLeftAutoManipulator();
            } else {
                robot.resetRightAutoManipulator();
            }
            // give it time to let go of the block:
            sleep(200);
            robot.moveRampToPosition(outside, .5, 16, robot, this, time);

        }
    }
}
