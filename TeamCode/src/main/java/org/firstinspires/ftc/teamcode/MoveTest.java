package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Deprecated
@Autonomous(name="RTP Test no function", group="test")
public class MoveTest extends LinearOpMode {

    HardwareDesignosaurs Robot = new HardwareDesignosaurs();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
        Robot.init(hardwareMap, 0, 0, 0);

        double encDist = 1120;
        /*
        Robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        Robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        Robot.backRight.setDirection(DcMotor.Direction.FORWARD);
        Robot.backLeft.setDirection(DcMotor.Direction.FORWARD);*/
        Robot.setTargetPos(Robot.frontRight, encDist);
        Robot.setTargetPos(Robot.frontLeft, encDist);
        Robot.setTargetPos(Robot.backLeft, encDist);
        Robot.setTargetPos(Robot.backRight, encDist);
        Robot.setPowers(Robot, .4);
        while (opModeIsActive()) {
            telemetry.addData("fr",Robot.frontRight.getCurrentPosition());
            telemetry.addData("fl",Robot.frontLeft.getCurrentPosition());
            telemetry.addData("br",Robot.backRight.getCurrentPosition());
            telemetry.addData("bl",Robot.backLeft.getCurrentPosition());
            telemetry.update();
        }
    }

}


