package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LeadScrewAdjustment", group="OurRobot")
public class LeadScrewAdjustment extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();                 // Use a OurRobot's hardware

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        robot.leadScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Say", "done with init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down || gamepad1.dpad_up){
                if (gamepad1.dpad_down) {
                    robot.leadScrew.setPower(-1);
                } else {
                    robot.leadScrew.setPower(0);
                }
                if (gamepad1.dpad_up) {
                    robot.leadScrew.setPower(1);
                } else {
                    robot.leadScrew.setPower(0);
                }
            } else {
                robot.leadScrew.setPower(0);
            }
            telemetry.addLine()
                    .addData("lead Screw", robot.leadScrew.getCurrentPosition());
            telemetry.update();
        }
    }
}