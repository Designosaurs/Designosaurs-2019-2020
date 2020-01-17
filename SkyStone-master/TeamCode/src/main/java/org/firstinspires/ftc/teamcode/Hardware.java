package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class Hardware {
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
    public Servo leftAutoManipulator = null;
    public Servo rightAutoManipulator = null;
    public Servo capstoneGripper    = null;

    public Servo mainGripperLeft    = null;
    public Servo mainGripperRight   = null;

    // Define Sensors
    public TouchSensor limitSwitch  = null;
    public ColorSensor sensorColor  = null;
    public DistanceSensor sensorDistance  = null;  // This is the one built into the color sensor.  Not accurate.
    public DistanceSensor sensorRange = null;      // The 2 Meter, accurate sensor.


    private ElapsedTime time = new ElapsedTime();

    // Define Variables
    enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        CW,
        CCW
    }
    public boolean flip = false;
    public double accelPerSec = .8;
    public double decelPGain = INCHES_PER_ENCODER_TICK / 45;
    public double minSpeed = .2;
    public double sideBias = Math.sqrt(2);

    public int power = 3;

    public static final double wheel_diameter = 4;   // inches
    public static final double encoder_ticks_per_revolution = 537.6;
    public static final double INCHES_PER_ENCODER_TICK =
                    (wheel_diameter * Math.PI)/
                            encoder_ticks_per_revolution; // used in encoder drive

    public static final float mmPerInch = 25.4f;

    HardwareMap hwMap = null;

    public Hardware() {

    }

    //////////////////////////////////////  INITIALIZATION ///////////////////////////////////////
    public void init2(HardwareMap hwMap) {
        // Initialize Motors
        // the FR & FL motors are intentionally switched due to a bug
        frontRight = hwMap.get(DcMotor.class,"front_left");
        frontLeft = hwMap.get(DcMotor.class,"front_right");
        backRight = hwMap.get(DcMotor.class,"back_right");
        backLeft = hwMap.get(DcMotor.class,"back_left");
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");

        // Initialize Servos
        mainGripperRight = hwMap.get(Servo.class, "right_manipulator");
        mainGripperLeft = hwMap.get(Servo.class, "left_manipulator");
        foundationGripper = hwMap.get(Servo.class, "foundation_manipulator");
        leftAutoManipulator = hwMap.get(Servo.class, "left_auto_manipulator");
        rightAutoManipulator = hwMap.get(Servo.class, "right_auto_manipulator");
        capstoneGripper = hwMap.get(Servo.class, "capstone_manipulator");

        // Initialize Sensors
        sensorRange = hwMap.get(DistanceSensor.class,"sensor_range");
        try {
            limitSwitch = hwMap.get(TouchSensor.class,"touch_sensor");

        } catch (Exception e) {}


        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        // 'distance' here is very short range, and no calibration at all.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        // Set Motor Directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Stop Motors
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


        // Configure Encoders
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Motors should brake at zero power
        frontLeft.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Servo Positions
        mainGripperLeft.setPosition(0.8);  // Open
        mainGripperRight.setPosition(.25);
        foundationGripper.setPosition(0.7);

        rightAutoManipulator.setPosition(1);
        capstoneGripper.setPosition(0);
        resetLeftAutoManipulator();
    }

    //////////////////  SERVO MANIPULATION //////////////////////////

    public void init2(HardwareMap hwMap, int xPos, int yPos, int thetaPos) {
        init2(hwMap);
    }

    public void resetLeftAutoManipulator (  ){
        leftAutoManipulator.setPosition(0);
    }

    public void deployLeftAutoManipulator(  ){
        leftAutoManipulator.setPosition(1);
    }


    ///////////////////////////////  SENSORS ////////////////////////////////////

    boolean seesYellow( LinearOpMode opMode ){
        int red = 0;
        red = sensorColor.red();
        opMode.telemetry.addData("Red", red );
        if ( red > 55 ) {
            return true;
        } else {
            return false;
        }
    }



    /////////////////////////////  MOVES /////////////////////////////////////////////
    // Drive until the proximity sensor (part of the color / prox sensor) shows less than the target value.
    // This is intended to be close so we go slow, and with encoders so we creep.
    // Return true if successful.
    boolean driveToProx( double targetProx, double TimeoutSecs, LinearOpMode opMode ){
        double startTime = time.now(TimeUnit.MILLISECONDS);
        double Prox = 1000;
        while (time.now(TimeUnit.MILLISECONDS) - startTime < TimeoutSecs * 1000 ) {
            runDirection(0.05, Direction.BACKWARD, true);
            Prox = sensorDistance.getDistance(DistanceUnit.CM);
            opMode.telemetry.addData("Doing", "driveToProx");
            opMode.telemetry.addData("Prox (cm)",
                    String.format(Locale.US, "%.01f", Prox ));
            opMode.telemetry.update();
            if (Prox < targetProx){
                stopDrive();
                return true;
            }
        }
        stopDrive();
        return false;
    }

    // Creep slowly until the color sensor detects a transition, denoting the edge of the
    // target on the skystone.
    // We look only for red as that appears to have the greatest contrast.
    // Return true with the robot stopped if we found the edge.
    // Return false with the robot stopped if we went the maxDistance without
    // seeing an edge.
    boolean driveToColorEdge( Direction direction, double maxDistance, boolean lookForYellow, LinearOpMode opMode ){

        double encDist = 0;
        int encReading = 0;
        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset all encoders.

        // Run until we see that edge, or maxDistane, whichever is first.
        do {
            runDirection(0.05, direction, true);

            opMode.telemetry.addData("Doing", "driveToColorEdge");
            encReading = frontLeft.getCurrentPosition();
            opMode.telemetry.addData("fl enc", encReading );
            encDist = Math.abs( ((double) encReading) * INCHES_PER_ENCODER_TICK);
            opMode.telemetry.addData("Dist ",
                    String.format(Locale.US, "%.01f", encDist ));

            // If we are looking for yellow, stop when we see it.
            if (lookForYellow){
                if ( seesYellow( opMode)) {
                    stopDrive();
                    opMode.telemetry.update();
                    return true;
                }
            } else {
                if ( !seesYellow( opMode)) {
                    stopDrive();
                    opMode.telemetry.update();
                    return true;
                }
            }
            opMode.telemetry.update();
        } while ( encDist < maxDistance );
        stopDrive();
        return false;
    }


    public void setTargetPos (DcMotor motor, double position) {
        // this function sets the specified motor to go to the specified position
        motor.setTargetPosition( (int) Math.round(position));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowers (Hardware Robot, double speed) {
        // this function sets all four drive motors to the specified power
        Robot.frontRight.setPower(speed);
        Robot.frontLeft.setPower(speed);
        Robot.backRight.setPower(speed);
        Robot.backLeft.setPower(speed);
    }

    public void stopDrive(  ){
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void moveRampToPosition(String direction, double maxSpeed, double distance, Hardware Robot, LinearOpMode opMode, ElapsedTime time) {
        // this function moves the specified number of inches in the given direction using acceleration ramps along with the built-in PIDs
        double encDist = distance / INCHES_PER_ENCODER_TICK; // calculate distance in encoder counts

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
        //ReadIMU until any of the motors are in position
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
            rampDownSpeed = Math.max(Math.abs(Math.abs(Robot.frontRight.getCurrentPosition()* INCHES_PER_ENCODER_TICK) - distance) * 1/10, minSpeed);
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
            opMode.telemetry.addData("ReadIMU ps", 1/deltaTime);
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

        opMode.telemetry.addData("RTP Status", "Done");
        opMode.telemetry.update();

    }

    // This does not actually work!
    // public void moveRampToPosition(Direction direction, double maxSpeed, double distance, Hardware Robot, LinearOpMode opMode, ElapsedTime time) {
    //    direction = maybeFlip(direction);
    //    moveRampToPosition(direction.toString().toLowerCase(), maxSpeed, distance, Robot, opMode, time);
    //}

    void moveDirection(double north, double west, double rotate, Hardware robot) {
        robot.frontLeft.setPower(-north + west + rotate);
        robot.frontRight.setPower(-north - west - rotate);
        robot.backLeft.setPower(north - west + rotate);
        robot.backRight.setPower(north + west - rotate);
    }

    void moveDirection(double north, double west, double rotate) {
        moveDirection(north,west,rotate,this);
    }

    //double getDistance() {
     //   return sensorRange.getDistance(DistanceUnit.INCH);
    //}

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




    // Drive until the proximity sensor (part of the color / prox sensor) shows less than the target value.
    // This is intended to be close so we go slow, and with encoders so we creep.
    // Return true if successful.
    void delaySecs( double  TimeoutSecs  ){
        double startTime = time.now(TimeUnit.MILLISECONDS);
        while (time.now(TimeUnit.MILLISECONDS) - startTime < TimeoutSecs * 1000 ) {
        }
        stopDrive();

    }


    void runDirection(double speed, Direction direction, boolean withEncoders ) {
        if ( withEncoders ){
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (direction == Direction.FORWARD) {
            frontLeft.setPower(-speed);
            frontRight.setPower(-speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

        } else if (direction == Direction.BACKWARD) {
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(-speed);
        } else if (direction == Direction.LEFT) {
            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);

        } else if (direction == Direction.RIGHT) {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
        }
    }

    void setPowers(double speed){
        setPowers(this, speed);
    }

    Direction flip(Direction direction) {
        if (direction == Direction.RIGHT) {
            return Direction.LEFT;
        } else if (direction == Direction.LEFT) {
            return Direction.RIGHT;
        } else {
            return direction;
        }
    }

    Direction maybeFlip(Direction direction) {
        if (flip) {
            return flip(direction);
        } else {
            return direction;
        }
    }

    double limit(double input, double max, double min) {
        return Math.max(Math.min(input,max),min);
    }

    double limit (double input, double max) {
        return limit(input,max,-max);
    }

    // Match Functions

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

}

