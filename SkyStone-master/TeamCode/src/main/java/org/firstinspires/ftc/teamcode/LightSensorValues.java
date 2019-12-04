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
                    .addData("green", robot.colorSensor.green());
            telemetry.addLine()
                    .addData("blue", robot.colorSensor.blue());
            telemetry.addLine()
                    .addData("red", robot.colorSensor.red());
            telemetry.addLine()
                    .addData("argb", robot.colorSensor.argb());
            telemetry.update();
        }
    }
}
