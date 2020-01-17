package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Test Moves", group = "!Auto")
public class TestsMoves extends LinearOpMode {
    ImuSubClass imu = new ImuSubClass();
    Hardware robot = new Hardware();

    ElapsedTime time = new ElapsedTime();

    boolean enabableStops = true; // Set to true to stop between steps for debugging.
    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critical parts.
    private void waitForYellow()  {
        if (!enabableStops) return;
        while( true ) {
            robot.stopDrive( );
            if (gamepad1.y) break;
            //updateSensors();
        }
        while( true ) {
            robot.stopDrive( );
            if (!gamepad1.y) break;
            // updateSensors();
        }
    }


    @Override
    public void runOpMode() {
        time.reset();
        robot.init2(hardwareMap,0,0,0);

        waitForStart();
        imu.ReadIMU();

        // Rotate the robot so the back (sensor / manipulator) side faces stones.
        imu.turnAndCorrect( -90, robot, this);
        waitForYellow();

        waitForStart();
        robot.moveRampToPosition("forward", .4,24,robot,this,time);
        waitForYellow();

        robot.moveRampToPosition("backward", .4,4,robot,this,time);
        waitForYellow();

        robot.moveRampToPosition("left", .4,4,robot,this,time);
        waitForYellow();

        robot.moveRampToPosition("backward", .4,1,robot,this,time);
        waitForYellow();

    }
}
