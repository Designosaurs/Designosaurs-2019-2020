package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ImuSubClass {
    BNO055IMU imu;
    Orientation angles;

    double heading = 0;

    public void init(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void ReadIMU() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public double getHeading() {
        return heading;
    }


    void turnSimp(double degrees, double speed, Hardware robot, LinearOpMode opMode) {
        ReadIMU();
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startHeading = getHeading();
        if (startHeading > degrees) {
            robot.moveDirection(0, 0, speed);
        } else {
            robot.moveDirection(0, 0, -speed);
        }
        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("target", degrees);
            opMode.telemetry.addData("current", getHeading());
            opMode.telemetry.update();

            ReadIMU();
            if (startHeading > degrees) {
                if (getHeading() < degrees) {
                    break;
                }
            } else {
                if (getHeading() > degrees) {
                    break;
                }
            }
        }
    }

    void turnAndCorrect(double degrees, Hardware robot, LinearOpMode opMode) {
        turnSimp(degrees, .4, robot, opMode);
        turnSimp(degrees, .2, robot, opMode);
        turnSimp(degrees, .05, robot, opMode);
    }

}
