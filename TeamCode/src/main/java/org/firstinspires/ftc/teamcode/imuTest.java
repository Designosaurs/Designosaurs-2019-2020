//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//
//import java.util.Locale;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "IMU Test OpMode", group = "IMU")
//public class imuTest extends LinearOpMode {
//
//    HardwareDesignosaurs Robot = new HardwareDesignosaurs();
//    private ElapsedTime runtime = new ElapsedTime();
//
//    double speed = .2;
//    double skewGain = 0.001;
//
//    @Override
//    public void runOpMode() {
//        telemetry.addData("init: ", "robot");
//        telemetry.update();
//        Robot.init(hardwareMap, 0, 0, 0);
//        telemetry.addData("init: ", "gyro");
//        telemetry.update();
//        Robot.initIMU();
//        telemetry.addData("init: ", "done");
//        telemetry.update();
//        waitForStart();
//
//        runtime.reset();
//        Robot.updateIMU();
//        double target = Robot.heading;
//        while (opModeIsActive()) {
//            Robot.updateIMU();
//            double skewComp = (target - Robot.heading) * skewGain;
//            Robot.frontLeft.setPower(-(speed) + skewComp);
//            Robot.frontRight.setPower(-(speed) - skewComp);
//            Robot.backLeft.setPower(speed + skewComp);
//            Robot.backRight.setPower(speed - skewComp);
//
//            telemetry.addData("Heading: ", Robot.angles.firstAngle);
//            telemetry.addData("Normal Heading: ", Robot.heading);
//            telemetry.addData("Roll: ", Robot.angles.secondAngle);
//            telemetry.addData("Pitch: ", Robot.angles.thirdAngle);
//            telemetry.update();
//        }
//        Robot.setPowers(Robot,0);
//
//    }
//
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }
//
//}
//
//
