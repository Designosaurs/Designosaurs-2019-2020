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

       double disOut;

    double lastTime;
    double deltTime;
    double currTime;

    double fireAtWill = 0;
    double imuTarget;

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
        imuTarget = imu.getHeading() + 90;

        //robot.moveRTP("right", .4,13.5,robot,this,time);
        //robot.moveRTP("forward", .4,2,robot,this,time);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.turnSimp(90,robot,this);
        //lockOn(false,false, .4);

        //robot.moveRTP(HardwareDesignosaurs.Direction.BACKWARD, .4, 6, robot, this, time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftGripper.setPosition(0);
        //robot.wait(1,this,time);

        //robot.moveRTP(HardwareDesignosaurs.Direction.LEFT, .6, 50,robot,this,time);
        //robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lockOn();
        //robot.moveRTP(HardwareDesignosaurs.Direction.RIGHT,.6,50,robot,this,time);
        //robot.leftGripper.setPosition(0);
        //robot.wait(1,this,time);
        //robot.moveRTP(HardwareDesignosaurs.Direction.LEFT,.6,10,robot,this,time);

    }


}
