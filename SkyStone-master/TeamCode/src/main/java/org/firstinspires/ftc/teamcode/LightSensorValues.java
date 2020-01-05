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

        /*    telemetry.addLine()
                    .addData("Alpha Right", robot.colorSensorRight.alpha());
            telemetry.addLine()
                    .addData("Alpha Left", robot.colorSensorLeft.alpha());
          */
            telemetry.addLine()
                    .addData("ARGB Right", robot.colorSensorRight.argb());
            telemetry.addLine()
                    .addData("ARGB Left", robot.colorSensorLeft.argb());
            telemetry.addLine()
                    .addData("Red Right", robot.colorSensorRight.red());
            telemetry.addLine()
                    .addData("Red Left", robot.colorSensorLeft.red());
            telemetry.addLine()
                    .addData("Green Right", robot.colorSensorRight.green());
            telemetry.addLine()
                    .addData("Green Left", robot.colorSensorLeft.green());
            telemetry.addLine()
                    .addData("Blue Right", robot.colorSensorRight.blue());
            telemetry.addLine()
                    .addData("Blue Left", robot.colorSensorLeft.blue());
            telemetry.update();
        }
    }
}
