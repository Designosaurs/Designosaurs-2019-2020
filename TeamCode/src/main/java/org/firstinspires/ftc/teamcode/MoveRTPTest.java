package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Deprecated
@Autonomous(name="RTP Test", group="test")
public class MoveRTPTest extends LinearOpMode {

    Hardware Robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
        Robot.init2(hardwareMap, 0, 0, 0);

        Robot.moveRampToPosition("forward", .4, 10.0 ,Robot, this, runtime);
        Robot.moveRampToPosition("left", .4, 10.0 ,Robot, this, runtime);
        Robot.moveRampToPosition("backward", .4, 10.0 ,Robot, this, runtime);
        Robot.moveRampToPosition("right", .4, 10.0 ,Robot, this, runtime);
    }

}


