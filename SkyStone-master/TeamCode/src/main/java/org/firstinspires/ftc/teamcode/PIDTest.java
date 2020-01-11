package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="PIDTest", group="OurRobot")
public class PIDTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();                 // Use a OurRobot's hardware

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .3, correction, rotation;
    PIDController           pidRotate, pidDrive;


    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        robot.init(hardwareMap, telemetry);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            telemetry.addData("Say", "waiting for calibrate");
            telemetry.update();
            sleep(50);
            idle();
        }

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);


        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Say", "done with init");
        telemetry.update();

        waitForStart();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();


        while (opModeIsActive()) {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

            correction = pidDrive.performPID(getAngle());

      /*      telemetry.addLine()
                    .addData("Y", angles.firstAngle);
            telemetry.addLine()
                    .addData("Z", angles.secondAngle);
            telemetry.addLine()
                    .addData("X", angles.thirdAngle);
            telemetry.addLine()
                    .addData("global", globalAngle);
   */
            telemetry.addLine()
                    .addData("correction", correction);
            telemetry.update();

            /*
            robot.rightFront.setPower(power - correction);
            robot.rightBack.setPower(power - correction);
            robot.leftFront.setPower(power + correction);
            robot.leftBack.setPower(power + correction);
            */
            rotate(90,0.3);
        }
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

      /*  telemetry.addData("globalAngle", globalAngle);
        telemetry.update();
*/
        return globalAngle;
    }
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.leftBack.setPower(power);
                robot.leftFront.setPower(power);
                robot.rightBack.setPower(power);
                robot.rightFront.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.leftBack.setPower(power);
                robot.leftFront.setPower(power);
                robot.rightBack.setPower(power);
                robot.rightFront.setPower(power);

            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.leftBack.setPower(power);
                robot.leftFront.setPower(power);
                robot.rightBack.setPower(power);
                robot.rightFront.setPower(power);

            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}
