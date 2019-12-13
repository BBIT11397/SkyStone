
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedSkyStone", group="bbit")
public class RedSkyStone extends LinearOpMode {
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
        robot.forward(2500,1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.strafeLeft(500, 1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "strafe");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        robot.forward(600, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(1000);

        double firstBlock = robot.colorSensor.alpha();
        telemetry.addLine()
                .addData("firstBlock", firstBlock);
        telemetry.update();

        robot.backward(600,1);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "backward");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
        sleep(1000);

        robot.strafeLeft(900, 0.5);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "strafe to next mineral");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
        sleep(500);

        //run into 2nd block
        robot.forward(800,0.5);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();
        sleep(500);

        double secondBlock = robot.colorSensor.alpha();
        telemetry.addLine()
                .addData("secondBlock", secondBlock);
        telemetry.update();

        boolean foundBlock = false;

        //decide what block is skystone
        if(firstBlock <= secondBlock && opModeIsActive()){
            if(firstBlock + 15 < secondBlock && opModeIsActive()) {
                //found skystone
                foundBlock = true;
                //back to first block

                robot.setUpMotors();
                //found sky stone
                robot.backward(1000,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.strafeRight(1300, 1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //open paddle top
                robot.paddleTop.setPosition(0.655);
                sleep(250);

                robot.forward(2800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);
                //close paddle top
                robot.paddleTop.setPosition(0.1);
                sleep(250);

                robot.backward(2800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //over line
                robot.strafeRight(4800, 1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.setUpMotors();
                robot.turnRight(1800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //open top paddle
                robot.paddleTop.setPosition(0.655);
                sleep(250);

                robot.backward(2300,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.paddleTop.setPosition(0.1);
                sleep(250);

            } else {

                robot.backward(600,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "backward");
                    telemetry.update();
                    idle();
                }
                robot.allMotorsStop();
                sleep(500);

                robot.strafeLeft(500, 0.5);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();

                sleep(250);

                //open paddle top
                robot.paddleTop.setPosition(0.655);
                sleep(250);

                robot.forward(2800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);
                //close paddle top
                robot.paddleTop.setPosition(0.1);
                sleep(250);

                robot.backward(3800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //over line
                robot.strafeRight(5800, 1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.setUpMotors();
                robot.turnRight(1800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //open top paddle
                robot.paddleTop.setPosition(0.655);
                sleep(250);

                robot.backward(1800,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.paddleTop.setPosition(0.1);
                sleep(250);

            }
        } else {
            if (secondBlock <= firstBlock && opModeIsActive()) {
                if (secondBlock + 15 < firstBlock && opModeIsActive()) {
                    //found sky stone
                    foundBlock = true;

                    robot.setUpMotors();
                    robot.backward(600,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "backward");
                        telemetry.update();
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(500);

                    robot.setUpMotors();
                    robot.strafeRight(300, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.backward(400,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //open paddle top
                    robot.paddleTop.setPosition(0.655);
                    sleep(250);

                    robot.forward(2800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);
                    //close paddle top
                    robot.paddleTop.setPosition(0.1);
                    sleep(250);

                    robot.backward(3000,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeRight(5500, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //drop off skystone
                    robot.setUpMotors();
                    robot.turnRight(1800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //open top paddle
                    robot.paddleTop.setPosition(0.655);
                    sleep(250);

                    robot.backward(1800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.paddleTop.setPosition(0.1);
                    sleep(250);

                } else {//strafe to final block]

                    robot.backward(600,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeLeft(500, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //found sky stone
                    robot.backward(400,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //open paddle top
                    robot.paddleTop.setPosition(0.655);
                    sleep(250);

                    robot.forward(2800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);
                    //close paddle top
                    robot.paddleTop.setPosition(0.1);
                    sleep(250);

                    robot.backward(3800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeRight(6200, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.setUpMotors();
                    robot.turnRight(1800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //open top paddle
                    robot.paddleTop.setPosition(0.655);
                    sleep(250);

                    robot.backward(1800,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.paddleTop.setPosition(0.1);
                    sleep(250);

                    robot.allMotorsStop();
                }
            }
        }
    }
}