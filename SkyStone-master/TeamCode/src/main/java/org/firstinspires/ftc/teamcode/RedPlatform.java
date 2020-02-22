package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="RedPlatform", group="bbit")
public class RedPlatform extends LinearOpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        telemetry.addData("Status", "Ready for Start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //FORWARD
        robot.backward(2500, .4);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(500);

        //slow to stones
        robot.backward(600, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(500);

        robot.foundationGrabberDown();
        sleep(250);

        robot.forward(600, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.forward(2000, .4);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.turnRight(5000, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.backward(2500, .4);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.foundationGrabberUp();
        sleep(250);

        robot.strafeRight(1500, .4);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.handUp();
        sleep(250);

        robot.turnLeft(500, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.forward(3900, .4);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
    }
}