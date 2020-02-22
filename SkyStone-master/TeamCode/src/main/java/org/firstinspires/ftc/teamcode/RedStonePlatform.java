package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="RedStonePlatform", group="bbit")
public class RedStonePlatform extends LinearOpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    double paddleUP = 0.8;
    double paddledown = 0;

    double power = 0.4;
    double power2 = 1;
    int backupb4reverse = 1200;
    int finalPark = 300;
    int reverseFromBlocks = 250;
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
        robot.forward(2500, 0.8);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(500);

        //slow to stones
        robot.forward(600, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(500);

        //read stones
        double firstBlock = robot.colorSensorRight.red();
        telemetry.addLine()
                .addData("Right", firstBlock);
        telemetry.update();

        double secondBlock = robot.colorSensorLeft.red();
        telemetry.addLine()
                .addData("Left", secondBlock);
        telemetry.update();

        //decide what block is skystone
        if(firstBlock <= secondBlock && opModeIsActive()){
            if(firstBlock + 2000 <= secondBlock && opModeIsActive()) {
                //found skystone
                //back to first block

                telemetry.addData("first", "first");
                telemetry.update();

                robot.backward(600, power);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.turnLeft(1900, power);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.backward(1250, power);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                captureSkystone();

                //over line
                robot.setUpMotors();
                robot.backward(4500, power2);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                releaseStonePlatformPark();
                robot.allMotorsStop();

            } else {

                //First;Third
                telemetry.addData("first", "third");
                telemetry.update();

                sleep(100);

                robot.backward(reverseFromBlocks + 300, power);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "backward");
                    telemetry.update();
                    idle();
                }
                robot.allMotorsStop();
                sleep(500);

                robot.turnLeft(1900, power);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();

                robot.forward(700, power);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();

                captureSkystone();

                //over line
                robot.setUpMotors();
                robot.backward(6100, power2);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                releaseStonePlatformPark();
                robot.allMotorsStop();
            }

        } else {
            if (secondBlock <= firstBlock && opModeIsActive()) {
                if (secondBlock + 2000 <= firstBlock && opModeIsActive()) {
                    //          Second;Second

                    telemetry.addData("second", "second");
                    telemetry.update();

                    sleep(100);

                    robot.setUpMotors();
                    robot.backward(500, power);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "backward");
                        telemetry.update();
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.turnLeft(1900, power);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();

                    robot.setUpMotors();
                    robot.forward(200, power);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    captureSkystone();

                    robot.setUpMotors();
                    robot.backward(5500, power2);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    releaseStonePlatformPark();
                    robot.allMotorsStop();

                } else {//strafe to final block]

                    telemetry.addData("second", "third");
                    telemetry.update();

                    robot.backward(reverseFromBlocks + 300, power);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "backward");
                        telemetry.update();
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(500);

                    robot.turnLeft(1900, power);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();

                    robot.forward(400, power);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();

                    captureSkystone();
                    sleep(250);

                    //over line
                    robot.setUpMotors();
                    robot.backward(6100, power2);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    releaseStonePlatformPark();
                    robot.allMotorsStop();
                }
            }
        }
    }

    public void right90(){
        robot.setUpMotors();
        robot.turnRight(1900, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
    }

    public void captureSkystone(){
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
        sleep(250);

        robot.strafeRight(1500, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);
    }

    public void backOnLine(){
        //open top paddle
        robot.fingerRelease();
        sleep(250);

        robot.handUp();
        sleep(500);

        robot.setUpMotors();
        robot.forward(2400, power2);
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

    public void releaseStonePlatformPark() {
        //open top paddle
        robot.fingerRelease();
        sleep(250);

        robot.handUp();
        sleep(500);

        robot.setUpMotors();
        robot.backward(1500, power2);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.setUpMotors();
        robot.turnLeft(1900, power);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.setUpMotors();
        robot.backward(1200, power2);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

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

        robot.forward(3575, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
/*
        robot.turnRight(5000, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "reline");
            idle();
        }
        robot.allMotorsStop();
        sleep(250);

        robot.backward(2500, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
*/
        robot.foundationGrabberUp();
        sleep(250);

        robot.strafeLeft(5300, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.forward(600, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

    }
}