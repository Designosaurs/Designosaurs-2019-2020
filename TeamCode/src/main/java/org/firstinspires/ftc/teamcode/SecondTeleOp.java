package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Second Robot Mecanum Drive", group="!TeleOp")
public class SecondTeleOp extends OpMode {

    // variables
    boolean isLowGear = false;


    HardwareDesignosaurs Robot = new HardwareDesignosaurs();
    private ElapsedTime runtime = new ElapsedTime();

    double lastTime = runtime.now(TimeUnit.MILLISECONDS);
    double deltaTime = 0;

    @Override
    public void init() {
        Robot.init2(hardwareMap); //initialize motors and sensors

        telemetry.addData("status: ", "Ready!");
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double fr, fl, bl, br; // Motor variables
        double lh, lv, rh;     // Joystick variables

        // calculate delta time
        deltaTime = runtime.now(TimeUnit.MILLISECONDS) - lastTime;
        lastTime = runtime.now(TimeUnit.MILLISECONDS);

        // set joystick variables
        lh = gamepad1.left_stick_x;
        lv = -gamepad1.left_stick_y;
        rh = gamepad1.right_stick_x;

        // square for exponential drive
        lh = Robot.square(lh, Robot.power);
        lv = Robot.square(lv, Robot.power);
        rh = Robot.square(rh, Robot.power);

        // calculate motor powers for mecanum drive
        fl = -lv + lh + rh;
        fr = -lv - lh - rh;
        bl = lv - lh + rh;
        br = lv + lh - rh;

        // set motor speeds
        if (isLowGear) {
            Robot.frontRight.setPower(fr/3);
            Robot.frontLeft.setPower(fl/3);
            Robot.backRight.setPower(br/3);
            Robot.backLeft.setPower(bl/3);
        } else {
            Robot.frontRight.setPower(fr);
            Robot.frontLeft.setPower(fl);
            Robot.backRight.setPower(br);
            Robot.backLeft.setPower(bl);
        }


        // set gripper location
        if (gamepad2.right_bumper){
            Robot.mainGripperLeft.setPosition(1);
            Robot.mainGripperRight.setPosition(.25);

        }

        if (gamepad2.left_bumper){
            Robot.mainGripperLeft.setPosition(.25);
            Robot.mainGripperRight.setPosition(1);

        }

        if (Robot.limitSwitch.isPressed() && !gamepad2.x) {
            Robot.liftMotor.setPower(Math.min(0,gamepad2.left_stick_y));
        } else if (Robot.liftMotor.getCurrentPosition() <= -4650 && !gamepad2.x) {
            Robot.liftMotor.setPower(Math.max(0,gamepad2.left_stick_y));
        } else {
            Robot.liftMotor.setPower(gamepad2.left_stick_y);
        }

        if (gamepad2.b) {
            if (Robot.limitSwitch.isPressed()){
                Robot.liftMotor.setPower(0);
                Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                Robot.liftMotor.setPower(.5);
            }
        }

        telemetry.addData("lift enc", Robot.liftMotor.getCurrentPosition());
        telemetry.addData("fr",Robot.frontRight.getCurrentPosition());
        telemetry.addData("fl",Robot.frontLeft.getCurrentPosition());
        telemetry.addData("br",Robot.backRight.getCurrentPosition());
        telemetry.addData("bl",Robot.backLeft.getCurrentPosition());
        telemetry.addData("touch",Robot.limitSwitch.isPressed());
        telemetry.update();

        if (gamepad2.dpad_up) {
            Robot.leftGripper.setPosition(0);
        } else if (gamepad2.dpad_down) {
            Robot.leftGripper.setPosition(1);
        }
        if (gamepad2.a) {
            Robot.rightGripper.setPosition(1);
        } else if (gamepad2.y) {
            Robot.rightGripper.setPosition(0);
        }

        // set position of foundation manipulator
        if (gamepad1.left_trigger > .5) {
            Robot.foundationGripper.setPosition(0.7);
        } else if (gamepad1.right_trigger > .5) {
            Robot.foundationGripper.setPosition(0.9);
        }
        if (gamepad1.left_bumper) {
            isLowGear = true;
        } else if (gamepad1.right_bumper) {
            isLowGear = false;
        }

        if (gamepad2.left_trigger > .5) {
            Robot.capstoneGripper.setPosition(0);
        } else if (gamepad2.right_trigger > .5) {
            Robot.capstoneGripper.setPosition(1);
        }
    }

    @Override
    public void stop() {
        telemetry.addData("status: ","Done!");
    }

}
