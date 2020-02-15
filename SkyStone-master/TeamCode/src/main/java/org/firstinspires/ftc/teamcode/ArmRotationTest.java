package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="ArmRotationTest", group="OurRobot")
public class ArmRotationTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();                 // Use a OurRobot's hardware

    public DcMotor armMotor;

    @Override
    public void runOpMode() {

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        armMotor.setPower(0);
        armMotor.setDirection(DcMotor.Direction.FORWARD);


        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Say", "done with init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                armMotor.setPower(-1);
            } else {
                armMotor.setPower(0);
            }

            if(gamepad1.dpad_up){
                armMotor.setPower(1);
            } else {
                armMotor.setPower(0);
            }
        }
    }
}