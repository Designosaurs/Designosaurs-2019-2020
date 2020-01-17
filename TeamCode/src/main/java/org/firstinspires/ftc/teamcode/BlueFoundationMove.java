package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue Foundation", group = "Mechanum")
public class BlueFoundationMove extends LinearOpMode {

    Hardware Robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
        double matchStartTime = runtime.now(TimeUnit.SECONDS);
        Robot.init2(hardwareMap, 0, 0, 0);

        Robot.moveRampToPosition("backward", .4, 24.0, Robot, this, runtime);
        Robot.moveRampToPosition("right", .4, 12.0, Robot, this, runtime);
        Robot.moveRampToPosition("backward", .15, 10.0, Robot, this, runtime);
        Robot.foundationGripper.setPosition(1);
        long startTime = runtime.time(TimeUnit.MILLISECONDS);
        while (Math.abs(runtime.time(TimeUnit.MILLISECONDS) - startTime) < 500 && opModeIsActive()) {
            telemetry.addData("time elapsed", runtime.time(TimeUnit.MILLISECONDS));
            telemetry.update();
        }
        Robot.moveRampToPosition("forward", .4, 34.0, Robot, this, runtime);
        Robot.foundationGripper.setPosition(0);
        startTime = runtime.time(TimeUnit.MILLISECONDS);
        while (Math.abs(runtime.time(TimeUnit.MILLISECONDS) - startTime) < 500 && opModeIsActive()) {
            telemetry.addData("time elapsed", runtime.time(TimeUnit.MILLISECONDS));
            telemetry.update();
        }
        //Robot.moveRampToPosition("forward", .4, 1.0 ,Robot, this, runtime);
        //Robot.moveRampToPosition("left", .4, 12 ,Robot, this, runtime);

        while (runtime.now(TimeUnit.SECONDS) - matchStartTime < 26 && opModeIsActive()) {
            telemetry.addData("time left:", 30 - runtime.time(TimeUnit.SECONDS));
            telemetry.update();
        }
        Robot.moveRampToPosition("left", .4, 36, Robot, this, runtime);
    }

}
