package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Jan Competition", group="!TeleOp")
public class TeleOpJan extends OpMode {

    // variables
    boolean isLowGear = false;


    Hardware Robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    double lastTime = runtime.now(TimeUnit.MILLISECONDS);
    double deltaTime = 0;

    @Override
    public void init() {
        Robot.init2(hardwareMap); //initialize motors and sensors

        telemetry.addData("status: ", "Ready!");
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.liftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double fr, fl, bl, br; // Motor variables
        double lh, lv, rh;     // Joystick variables

        // calculate delta time
        deltaTime = runtime.now(TimeUnit.MILLISECONDS) - lastTime;
        lastTime = runtime.now(TimeUnit.MILLISECONDS);

        ////////////  RUNNNER //////////////////////////
        if (gamepad1.left_bumper) {
            isLowGear = true;
        } else if (gamepad1.right_bumper) {
            isLowGear = false;
        }

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

        // set position of foundation manipulator
        if (gamepad1.left_trigger > .5) {
            Robot.foundationGripper.setPosition(0.7);
        } else if (gamepad1.right_trigger > .5) {
            Robot.foundationGripper.setPosition(0.9);
        }

        /////////////////  GUNNER ////////////////////////
        //  Open the gripper
        if (gamepad2.right_bumper){
            Robot.mainGripperLeft.setPosition(0.8);
            Robot.mainGripperRight.setPosition(.25);

        }
        // Close the gripper
        if (gamepad2.right_trigger > 0.5 ){
            Robot.mainGripperLeft.setPosition(.25);
            Robot.mainGripperRight.setPosition(1);
        }

        // Lift Control- Joystick.
        // Lift up => negative values of encoder.
        // Negative power => ift up.
        // Gamepad X functions as an override limit.
        if (Robot.limitSwitch.isPressed() ) {
            // Limit switch is pressed, so may only go up.
            Robot.liftMotor.setPower(Math.min(0,gamepad2.left_stick_y));
        } else if (Robot.liftMotor.getCurrentPosition() <= -4650 ) {
            // Hit endocder limit.  So, may only go down.
            Robot.liftMotor.setPower(Math.max(0,gamepad2.left_stick_y));
        } else {
            // Normal operation.  Not hitting any limit.
            // Set the power according to where it is.
            if (Robot.liftMotor.getCurrentPosition() < 1000 ) {
                // Normal operation limit motor to 70%.  Don't break the hardware!
                Robot.liftMotor.setPower(gamepad2.left_stick_y * 0.7);
            } else {
                // Near zero slow down.
                Robot.liftMotor.setPower(gamepad2.left_stick_y * 0.2);
            }
        }

        // Lift Control- B button: Descend and reset zero. is decsencd and reeset zero.
        if (gamepad2.b) {
            if (Robot.limitSwitch.isPressed()){
                Robot.liftMotor.setPower(0);
                Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                Robot.liftMotor.setPower(.2);
            }
        }

        // Capstone.  Left Trigger Deploys, Bumper resets
        if (gamepad2.left_trigger > .5) {
            Robot.capstoneGripper.setPosition(1);
        } else if (gamepad2.left_bumper) {
            Robot.capstoneGripper.setPosition(0.5);
        }

        telemetry.addData("lift enc", Robot.liftMotor.getCurrentPosition());
        telemetry.addData("Lift",Robot.liftMotor.getPower());
        telemetry.addData("fr",Robot.frontRight.getCurrentPosition());
        telemetry.addData("fl",Robot.frontLeft.getCurrentPosition());
        telemetry.addData("br",Robot.backRight.getCurrentPosition());
        telemetry.addData("bl",Robot.backLeft.getCurrentPosition());
        telemetry.addData("touch",Robot.limitSwitch.isPressed());
        telemetry.update();



    }

    @Override
    public void stop() {
        telemetry.addData("status: ","Done!");
    }

}
