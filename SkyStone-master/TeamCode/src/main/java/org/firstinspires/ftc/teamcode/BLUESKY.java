package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="BLUESKY", group="bbit")
public class BLUESKY extends LinearOpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private static final String VUFORIA_KEY = "AR6vl4b/////AAABmZgsD/qFKU5ckqZbY3OoUJIcRxE+StRnHOz7Rbpuii+XYylNWzTn29Q/ZbmfRiIJb5Oy4w65MdCo7yrGyOdpKQqCvBfoEZ4ykwXruAnjmW+rXCoatYZ+L1393f/YFuplJUKjbWUbLj45ygMcCPEZ8DNB9JnOZeZxiLygd6k6VVB8YVGHMReVj9vRYJyJj8UV+xknITZGgZ6QPC4IBI/Gh3uWHen96+cH28fJPsyVss1jcdNGCZKunj6io8oWmA6katkVdHfNyH/YWOHOIhkMP1LdUeL2WlbFa0nRbEIvDSVfCXAo5IdnXD8HsESVlZPpKdQzrUVuGo3kfd6ny5yHNCzQU4krKnuqNgwl0uEwpGdp";
    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private OpenGLMatrix lastLocation = null;
    private static final float mmPerInch = 25.4f;
    private static final float stoneZ = 2.00f * mmPerInch;
    private String name;

    boolean isSkyStoneVisible = false;

    double power = 0.4;
    int finalPark = 900;
    int reverseFromBlocks = 250;
    int stonePosition = -1;

    float yValue = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = true;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        targetsSkyStone.activate();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        telemetry.addData("Status", "Ready for Start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //FORWARD
        robot.forward(500, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.strafeRight(800, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.forward(1500, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(500);

        stonePosition = getPositionNumber(stoneTarget);

        targetsSkyStone.deactivate();

        //slow to stones
        robot.forward(600, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.setUpMotors();
        robot.turnLeft(1850, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();

        //decide what block to grab

        if (stonePosition == 6 && opModeIsActive()) {
            getStone6();
        }

        if (stonePosition == 5 && opModeIsActive()) {
            getStone5();
        }

        if (stonePosition == 4 && opModeIsActive()) {
            getStone4();
        }
    }

    public void right90() {
        robot.setUpMotors();
        robot.turnRight(1850, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
    }

    public void captureSkystone() {
        robot.handDown();
        sleep(500);

        robot.strafeLeft(700, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.fingerGrab();
        sleep(500);

        robot.strafeRight(1500, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);
    }

    public void backOnLine() {
        //open top paddle
        robot.fingerRelease();
        sleep(250);

        robot.handUp();
        sleep(250);

        robot.setUpMotors();
        robot.backward(2400, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.strafeLeft(finalPark, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
    }

    public void getStone6() {
        //found skystone
        //back to first block

        telemetry.addData("first", "first");
        telemetry.update();

       /* robot.forward(800, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        */
        captureSkystone();

        //over line
        robot.setUpMotors();
        robot.forward(4500, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        backOnLine();
        robot.allMotorsStop();
    }

    public void getStone4() {
        //First/Second ;Third
        telemetry.addData("first", "third");
        telemetry.update();

        sleep(100);

       /*robot.forward(800, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        */
        captureSkystone();

        //over line
        robot.setUpMotors();
        robot.forward(6100, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        backOnLine();
        robot.allMotorsStop();
    }

    public void getStone5() {
        telemetry.addData("second", "second");
        telemetry.update();

        sleep(100);

        robot.setUpMotors();
        robot.forward(100, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        captureSkystone();

        robot.setUpMotors();
        robot.forward(5500, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        backOnLine();
        robot.allMotorsStop();
    }

    public boolean skystoneNotFound(VuforiaTrackable stoneTarget) {

        ElapsedTime vTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        vTimer.reset();

        lastLocation = null;

        while (!isSkyStoneVisible && opModeIsActive()) {
            //is stone in front of camera
            name = stoneTarget.getName();
            telemetry.addData("FIX", name);
            telemetry.update();
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                telemetry.update();
                isSkyStoneVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            if (vTimer.milliseconds() > 1000) {
                break;
            }
            //do some stuff
        }
        return isSkyStoneVisible;
    }

    public int getPositionNumber(VuforiaTrackable stoneTarget) {
        if (skystoneNotFound(stoneTarget)) {
            return 6;
        }

        robot.strafeLeft(900, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();

        if (skystoneNotFound(stoneTarget)) {
            return 5;
        }

        robot.strafeLeft(650, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();

        return 4;
    }
}