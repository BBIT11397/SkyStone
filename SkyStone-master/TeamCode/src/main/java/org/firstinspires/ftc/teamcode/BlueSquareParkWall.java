package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="BlueSquareParkWall", group="bbit")
    public class BlueSquareParkWall extends LinearOpMode {

        /* Declare OpMode members. */
        Hardware robot = new Hardware();
        private ElapsedTime runtime = new ElapsedTime();

        static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        public int highestRightBackPosition;
        public int highestRightFrontPosition;
        public int highestLeftFrontPosition;
        public int highestLeftBackPosition;

        @Override
        public void runOpMode() {
            robot.init(hardwareMap, telemetry);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.leftBack.getCurrentPosition(),
                    robot.rightBack.getCurrentPosition(),
                    robot.leftFront.getCurrentPosition(),
                    robot.rightFront.getCurrentPosition());
            telemetry.update();

            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
/*
             double floor = robot.colorSensor.blue();

            highestRightBackPosition = robot.rightBack.getCurrentPosition() + 4000;
            highestRightFrontPosition = robot.rightFront.getCurrentPosition() + 4000;
            highestLeftFrontPosition = robot.leftFront.getCurrentPosition() + 4000;
            highestLeftBackPosition = robot.leftBack.getCurrentPosition() + 4000;

            while (floor <= 400 && opModeIsActive()) {
                telemetry.addLine()
                        .addData("blue", robot.colorSensor.blue());
                telemetry.update();

                robot.rightBack.setPower(0.5);
                robot.rightFront.setPower(-0.5);
                robot.leftFront.setPower(0.5);
                robot.leftBack.setPower(-0.5);

                if (robot.leftBack.getCurrentPosition() > highestLeftBackPosition){
                    robot.allMotorsStop();
                }
            }

 */
            setUpMotors();
            robot.leftFront.setTargetPosition(3500);
            robot.leftBack.setTargetPosition(3500);
            robot.rightBack.setTargetPosition(3500);
            robot.rightFront.setTargetPosition(3500);

            robot.rightBack.setPower(0.5);
            robot.rightFront.setPower(-0.5);
            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(-0.5);

            while (robot.checkMotorIsBusy() && opModeIsActive()) {
                telemetry.addLine()
                        .addData("Task", "Wait");
                telemetry.update();
                idle();
            }
            sleep(250);

            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.allMotorsStop();

            telemetry.addLine()
                    .addData("path", "complete");
            telemetry.update();
        }

        public void setUpMotors() {
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }