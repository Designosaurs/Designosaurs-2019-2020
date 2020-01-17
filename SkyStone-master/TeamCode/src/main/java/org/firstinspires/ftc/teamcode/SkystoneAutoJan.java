package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Jan Skystone Auto", group = "!Auto")
public class SkystoneAutoJan extends LinearOpMode {
    ImuSubClass imu = new ImuSubClass();
    Hardware robot = new Hardware();
    ElapsedTime time = new ElapsedTime();

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

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
        telemetry.addData("selected", "Blue");
        telemetry.update();

        telemetry.addData("init","Imu");
        imu.init(hardwareMap);
        telemetry.addData("init","Robot");
        telemetry.update();
        robot.init2(hardwareMap,0,0,0);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        imu.ReadIMU();

        // Creep forward to get within range of color sensor.
        robot.driveToProx( 8.0, 3.0, this);
        waitForYellow();

        // Creep until get into the Yellow.
        robot.driveToColorEdge( Hardware.Direction.LEFT, 6.0, true ,this);
        waitForYellow();

        // Back up to get the grabber centered.
        robot.moveRampToPosition( Hardware.Direction.RIGHT, .4,8,robot,this,time);
        waitForYellow();

        // Backward (toward stone)  to be ready to grab that stone.
        robot.moveRampToPosition( Hardware.Direction.BACKWARD, .4,1.5,robot,this,time);
        waitForYellow();


        // Deploy manipulator
        robot.deployLeftAutoManipulator();
        waitForYellow();

         while( true ) {

             Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                     (int) (robot.sensorColor.green() * SCALE_FACTOR),
                     (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                     hsvValues);

             // send the info back to driver station using telemetry function.
             telemetry.addData("Alpha", robot.sensorColor.alpha());
             telemetry.addData("Red  ", robot.sensorColor.red());
             telemetry.addData("Green", robot.sensorColor.green());
             telemetry.addData("Blue ", robot.sensorColor.blue());
             telemetry.addData("Hue", hsvValues[0]);

             telemetry.addData("Prox (cm)",
                     String.format(Locale.US, "%.01f", robot.sensorDistance.getDistance(DistanceUnit.CM)));

            telemetry.addData( "Range (In)", "%.01f", robot.sensorRange.getDistance( DistanceUnit.INCH ));

            // telemetry.addData("Dist",robot.getDistance());
            telemetry.update();
             if (gamepad1.a) break;
            //updateSensors();
        }
        while( true ) {
            robot.stopDrive( );
            if (!gamepad1.a) break;
            // updateSensors();
        }


        //robot.moveRampToPosition("right", .4,13.5,robot,this,time);
        robot.moveRampToPosition("forward", .4,24,robot,this,time);
        waitForYellow();
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnSimp(90,robot,this);
        //lockOn(false,false, .4);

        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.BACKWARD, .4, 6, robot, this, time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftAutoManipulator.setPosition(0);
        //robot.wait(1,this,time);

        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.LEFT, .6, 50,robot,this,time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lockOn();
        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.RIGHT,.6,50,robot,this,time);
        //robot.leftAutoManipulator.setPosition(0);
        //robot.wait(1,this,time);
        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.LEFT,.6,10,robot,this,time);

    }


}
