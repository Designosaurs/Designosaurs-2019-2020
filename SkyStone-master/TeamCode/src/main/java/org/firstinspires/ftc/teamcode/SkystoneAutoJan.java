package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Jan Skystone Auto", group = "!Auto")
public class SkystoneAutoJan extends LinearOpMode {
    ImuSubClass imu = new ImuSubClass();
    HardwareDesignosaursJan robot = new HardwareDesignosaursJan();
    ElapsedTime time = new ElapsedTime();



    boolean enabableStops = true; // Set to true to stop between steps for debugging.
    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critical parts.
    private void waitForGreen()  {
        if (!enabableStops) return;
        while( true ) {
            robot.stopDrive( robot );
            if (gamepad1.a) break;
            //updateSensors();
        }
        while( true ) {
            robot.stopDrive( robot );
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


         while( true ) {
            telemetry.addData("Dist",robot.getDistance());
            telemetry.update();
             if (gamepad1.a) break;
            //updateSensors();
        }
        while( true ) {
            robot.stopDrive( robot );
            if (!gamepad1.a) break;
            // updateSensors();
        }


        //robot.moveRampToPosition("right", .4,13.5,robot,this,time);
        robot.moveRampToPosition("forward", .4,24,robot,this,time);
        waitForGreen();
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnSimp(90,robot,this);
        //lockOn(false,false, .4);

        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.BACKWARD, .4, 6, robot, this, time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftGripper.setPosition(0);
        //robot.wait(1,this,time);

        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.LEFT, .6, 50,robot,this,time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lockOn();
        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.RIGHT,.6,50,robot,this,time);
        //robot.leftGripper.setPosition(0);
        //robot.wait(1,this,time);
        //robot.moveRampToPosition(HardwareDesignosaurs.Direction.LEFT,.6,10,robot,this,time);

    }


}
