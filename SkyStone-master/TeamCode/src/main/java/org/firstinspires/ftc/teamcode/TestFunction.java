package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
    @Autonomous(name="TestFunction", group="bbit")
    public class TestFunction extends LinearOpMode {

        /* Declare OpMode members. */
        Hardware robot = new Hardware();
        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {
            robot.init(hardwareMap, telemetry);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();
            telemetry.addData("Status","Ready for Start");

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            robot.setUpMotors();
            robot.turnLeft(1800,1);
            while (robot.checkMotorIsBusy() && opModeIsActive()) {
                telemetry.addLine()
                        .addData("Task", "Wait");
                telemetry.update();
                idle();
            }
            sleep(250);

            telemetry.addLine()
                    .addData("path", "complete");
            telemetry.update();
        }
    }
