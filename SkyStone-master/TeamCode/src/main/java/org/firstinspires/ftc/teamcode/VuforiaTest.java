package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="VuforiaTest", group="bbit")

public class VuforiaTest extends LinearOpMode {

    private static final String VUFORIA_KEY = "AR6vl4b/////AAABmZgsD/qFKU5ckqZbY3OoUJIcRxE+StRnHOz7Rbpuii+XYylNWzTn29Q/ZbmfRiIJb5Oy4w65MdCo7yrGyOdpKQqCvBfoEZ4ykwXruAnjmW+rXCoatYZ+L1393f/YFuplJUKjbWUbLj45ygMcCPEZ8DNB9JnOZeZxiLygd6k6VVB8YVGHMReVj9vRYJyJj8UV+xknITZGgZ6QPC4IBI/Gh3uWHen96+cH28fJPsyVss1jcdNGCZKunj6io8oWmA6katkVdHfNyH/YWOHOIhkMP1LdUeL2WlbFa0nRbEIvDSVfCXAo5IdnXD8HsESVlZPpKdQzrUVuGo3kfd6ny5yHNCzQU4krKnuqNgwl0uEwpGdp";
    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private OpenGLMatrix lastLocation = null;
    private static final float mmPerInch        = 25.4f;
    private static final float stoneZ = 2.00f * mmPerInch;
    private String  name;

    boolean isSkyStoneVisible = false;

    @Override
    public void runOpMode() {
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

        waitForStart();

        while (!isSkyStoneVisible) {
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
            if (isSkyStoneVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0), translation.get(1), translation.get(2));
                telemetry.update();

            }
            //do some stuff

        }
        sleep(15000);
        targetsSkyStone.deactivate();
    }

}
