package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LightSensorValues", group="OurRobot")
public class LightSensorValues extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();                 // Use a OurRobot's hardware

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        telemetry.addData("Say", "done with init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine()
                    .addData("Alpha Right", robot.colorSensorRight.alpha());
            telemetry.addLine()
                    .addData("Alpha Left", robot.colorSensorLeft.alpha());
            telemetry.update();
        }
    }
}
