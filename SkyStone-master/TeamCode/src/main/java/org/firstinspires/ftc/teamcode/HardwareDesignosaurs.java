package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

public class HardwareDesignosaurs {

    // Define Motors
    public DcMotor frontRight = null;
    public DcMotor frontLeft  = null;
    public DcMotor backRight  = null;
    public DcMotor backLeft   = null;
    public DcMotor pitchMotor = null;
    public DcMotor liftMotor  = null;

    // Define Servos
    public Servo mainGripper        = null;
    public Servo foundationGripper  = null;
    public Servo leftGripper        = null;
    public Servo rightGripper       = null;
    public Servo capstoneGripper    = null;

    // Define Sensors
    public DistanceSensor distance = null;
//    public BNO055IMU imu = null;
//    Orientation angles = null;
//    Acceleration gravity;
//    double heading = 0;

    // Define Variables
    public double startEncoder;
    public double lastTime = 0;
    public double deltaTime = 0;
    public double speed = 0;
    public double accelPerSec = .8;
    public double decelPGain = encoder_ticks_per_inch / 45;
    public double minSpeed = .2;
    public double sideBias = Math.sqrt(2);

    public int power = 3;

    public static final double wheel_diameter = 4;   // inches
    public static final double encoder_ticks_per_revolution = 537.6;
    public static final double encoder_ticks_per_inch =
                    (wheel_diameter * Math.PI)/
                            encoder_ticks_per_revolution; // used in encoder drive

    private static final String VUFORIA_KEY =
            "AdCuaEX/////AAABmXYJgRHZxkB9gj+81cIaX+JZm4W2w3Ee2HhKucJINnuXQ8l214BoCiyEk04zmQ/1VPvo+8PY7Um3eI5rI4WnSJmEXo7jyMz2WZDkkRnA88uBCtbml8CsMSIS7J3aUcgVd9P8ocLLgwqpavhEEaUixEx/16rgzIEtuHcq5ghQzzCkqR1xvAaxnx5lWM+ixf6hBCfZEnaiUM7WjD4gflO55IpoO/CdCWQrGUw2LuUKW2J+4K6ftKwJ+B1Qdy7pt2tDrGZvMyB4AcphPuoJRCSr5NgRoNWZ+WH5LqAdzYEO0Bv7C9LeSgmSPPT7GPPDpjv6+3DO5BE6l+2uMYQQbuF11BWKKq5Xp+D5Y6l2+W97zpgP";
    public static final float mmPerInch = 25.4f;

    HardwareMap hwMap = null;

    public HardwareDesignosaurs() {

    }
  
    public void init(HardwareMap ahwMap, int xPos, int yPos, int thetaPos) {
        hwMap = ahwMap;

        // Initialize Motors
        frontRight = hwMap.get(DcMotor.class,"front_right");
        frontLeft = hwMap.get(DcMotor.class,"front_left");
        backRight = hwMap.get(DcMotor.class,"back_right");
        backLeft = hwMap.get(DcMotor.class,"back_left");
        pitchMotor = hwMap.get(DcMotor.class, "pitch_motor");
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");

        // Initialize Servos
        mainGripper = hwMap.get(Servo.class, "main_manipulator");
        foundationGripper = hwMap.get(Servo.class, "foundation_manipulator");
        leftGripper = hwMap.get(Servo.class, "left_auto_manipulator");
        rightGripper = hwMap.get(Servo.class, "right_auto_manipulator");
        capstoneGripper = hwMap.get(Servo.class, "capstone_manipulator");


        // Initialize Sensors
        distance = hwMap.get(DistanceSensor.class,"sensor_range");

//        imu = hwMap.get(BNO055IMU.class,"imu");


        // Set Motor Directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Stop Motors
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        pitchMotor.setPower(0.7);
        pitchMotor.setTargetPosition(0);

        // Enable All Encoders
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Servo Positions

        mainGripper.setPosition(1);
        foundationGripper.setPosition(0.7);
        leftGripper.setPosition(0);
        rightGripper.setPosition(1);
        capstoneGripper.setPosition(0);

    }

    // Functions
//    public void initIMU() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu.initialize(parameters);
//
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//    }
//
//
//    public void updateIMU() {
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
//    }


    public double square(double base, int power) { //function that does fractional squaring
        if (base == 0) {
            return 0;
        } else {
            double basen, based;
            basen = base; // numerator
            based = base; // denominator

            // Multiplies numerator
            for (int i = 0; i < (power+16); i++) {
                basen *= base;
            }

            // Multiplies denominator
            for (int i = 0; i < 16; i++) {
                based *= base;
            }
            return basen/based;

        }
    }

    public void setTargetPos (DcMotor motor, double position) {
        // this function sets the specified motor to go to the specified position
        motor.setTargetPosition( (int) Math.round(position));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowers (HardwareDesignosaurs Robot, double speed) {
        // this function sets all four drive motors to the specified power
        Robot.frontRight.setPower(speed);
        Robot.frontLeft.setPower(speed);
        Robot.backRight.setPower(speed);
        Robot.backLeft.setPower(speed);
    }

    public void moveRTP(String direction, double maxSpeed, double distance, HardwareDesignosaurs Robot, LinearOpMode opMode, ElapsedTime time) {
        // this function moves the specified number of inches in the given direction using acceleration ramps along with the built-in PIDs
        double encDist = distance / encoder_ticks_per_inch; // calculate distance in encoder counts

        Robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset all encoders
        Robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target positions based on direction
        if (direction == "forward") {
            setTargetPos(Robot.frontLeft, -encDist);
            setTargetPos(Robot.frontRight, -encDist);
            setTargetPos(Robot.backLeft, encDist);
            setTargetPos(Robot.backRight, encDist);

        } else if (direction == "backward") {
            setTargetPos(Robot.frontLeft, encDist);
            setTargetPos(Robot.frontRight, encDist);
            setTargetPos(Robot.backLeft, -encDist);
            setTargetPos(Robot.backRight, -encDist);
        } else if (direction == "left") {
            setTargetPos(Robot.frontLeft, -encDist * sideBias);
            setTargetPos(Robot.frontRight, encDist * sideBias);
            setTargetPos(Robot.backLeft, encDist * sideBias);
            setTargetPos(Robot.backRight, -  encDist * sideBias);

        } else if (direction == "right") {
            setTargetPos(Robot.frontLeft, encDist * sideBias);
            setTargetPos(Robot.frontRight, -encDist * sideBias);
            setTargetPos(Robot.backLeft, -encDist * sideBias);
            setTargetPos(Robot.backRight, encDist * sideBias);
        } else {
            opMode.telemetry.addData("failure","invalid input");
            opMode.telemetry.update();
        }

        // delta time setup
        time.reset();
        double prevTime = time.time();
        double nowTime;
        double deltaTime;
        double rampUpSpeed = 0;
        double rampDownSpeed = 0;
        double currentSpeed = 0;
        boolean rampDownLock = false;
        //loop until any of the motors are in position
        while ((Robot.frontRight.isBusy() && Robot.frontLeft.isBusy() && Robot.backRight.isBusy() && Robot.backLeft.isBusy()) && opMode.opModeIsActive()) {
            // calculate rampup, delta time * accel var
            nowTime = time.time();
            deltaTime = nowTime - prevTime;
            prevTime = nowTime;
            if (rampUpSpeed < maxSpeed) {
                rampUpSpeed += deltaTime * accelPerSec;
            } else {
                rampUpSpeed = maxSpeed;
            }
            // calculate rampdown, remaining distance / 10, no less than min var
            rampDownSpeed = Math.max(Math.abs(Math.abs(Robot.frontRight.getCurrentPosition()*encoder_ticks_per_inch) - distance) * 1/10, minSpeed);
            // lock ramp down at min var
            if (rampDownLock) {
                rampDownSpeed = minSpeed;
            } else if (minSpeed == rampDownSpeed) {
                rampDownLock = true;
            }

            //set powers to whatever is lower, rampup or rampdown
            currentSpeed = Math.min(rampUpSpeed,rampDownSpeed);
            setPowers(Robot, currentSpeed);

            //display debug info
            opMode.telemetry.addData("fr",Robot.frontRight.getCurrentPosition());
            opMode.telemetry.addData("fl",Robot.frontLeft.getCurrentPosition());
            opMode.telemetry.addData("br",Robot.backRight.getCurrentPosition());
            opMode.telemetry.addData("bl",Robot.backLeft.getCurrentPosition());
            opMode.telemetry.addData("loop ps", 1/deltaTime);
            opMode.telemetry.addData("ramp up speed", rampUpSpeed);
            opMode.telemetry.addData("ramp down speed", rampDownSpeed);
            opMode.telemetry.addData("target speed", currentSpeed);
            opMode.telemetry.update();
        }
        // wait for 500ms so PIDs can settle
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 500 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("time until quit", 500 - time.time(TimeUnit.MILLISECONDS));
            opMode.telemetry.update();
        }
        //stop robot
        setPowers(Robot, 0);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    void moveDirection(double north, double west, double rotate, HardwareDesignosaurs robot) {
        robot.frontLeft.setPower(-north + west + rotate);
        robot.frontRight.setPower(-north - west - rotate);
        robot.backLeft.setPower(north - west + rotate);
        robot.backRight.setPower(north + west - rotate);
    }

    double getDistance() {
        return distance.getDistance(DistanceUnit.INCH);
    }

    void wait(double seconds, LinearOpMode opMode, ElapsedTime time) {
        double startTime = time.now(TimeUnit.MILLISECONDS);
        while (time.now(TimeUnit.MILLISECONDS) - startTime < seconds * 1000 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("wait","ing");
            opMode.telemetry.update();
        }
    }

    void setMode(DcMotor.RunMode mode){
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }
}

