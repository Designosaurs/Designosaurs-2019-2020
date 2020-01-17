package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "FoundationAutoJan", group = "!Auto")
public class FoundationAutoJan extends LinearOpMode {
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
    private void waitForGreen()  {
        if (!enabableStops) return;
        while( true ) {
            robot.stopDrive( );
            if (gamepad1.a) break;
            //updateSensors();
        }
        while( true ) {
            robot.stopDrive( );
            if (!gamepad1.a) break;
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

        //robot.driveToColorEdge( Hardware.Direction.LEFT, 6.0, this);
        //waitForGreen();

        //robot.driveToProx( 10.0, 3.0, this);
        //waitForGreen();



        /*
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


         */


        //robot.moveRampToPosition("right", .4,13.5,robot,this,time);
        robot.foundationGripper.setPosition(0.7);
        robot.moveRampToPosition("left", .4,4,robot,this,time);
        //waitForGreen();
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnSimp(-90,robot,this);
        robot.moveRampToPosition("right", .4,16,robot,this,time);
        //imu.turnSimp(-90,robot,this);
        sleep((100));
        robot.moveRampToPosition("backward", .4,20,robot,this,time);
        robot.moveRampToPosition("backward", .1,5,robot,this,time);
        robot.foundationGripper.setPosition(0.9);
        sleep((100));
        robot.moveRampToPosition("forward", .4,30,robot,this,time);
        robot.foundationGripper.setPosition(0.7);
        robot.moveRampToPosition("left", .4,40,robot,this,time);

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
