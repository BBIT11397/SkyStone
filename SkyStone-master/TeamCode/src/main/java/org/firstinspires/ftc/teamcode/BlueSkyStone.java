
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BlueSkyStone", group="bbit")
public class BlueSkyStone extends LinearOpMode {
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

        robot.forward(600, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(1000);

        double firstBlock = robot.colorSensorLeft.alpha();
        telemetry.addLine()
                .addData("Left", firstBlock);
        telemetry.update();

        double secondBlock = robot.colorSensorRight.alpha();
        telemetry.addLine()
                .addData("Right", secondBlock);
        telemetry.update();

        boolean foundBlock = false;

        //decide what block is skystone
        if(firstBlock <= secondBlock && opModeIsActive()){
            if(firstBlock + 10000 < secondBlock && opModeIsActive()) {
                //found skystone
                foundBlock = true;
                //back to first block

                //open paddle top
                robot.paddleTop.setPosition(0.655);
                sleep(250);

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

                robot.forward(1800,1);
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

                robot.backward(2000,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //over line
                robot.strafeLeft(4500, 1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.setUpMotors();
                robot.turnLeft(1800,1);
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

                robot.backward(2400,1);
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

                robot.strafeRight(1100, 0.5);
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

                robot.forward(2500,1);
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

                robot.backward(2500,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //over line
                robot.strafeLeft(5800, 1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                robot.setUpMotors();
                robot.turnLeft(1800,1);
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
                if (secondBlock + 10000 < firstBlock && opModeIsActive()) {
                    //found sky stone
                    foundBlock = true;

                    robot.setUpMotors();

                    //open paddle top
                    robot.paddleTop.setPosition(0.655);
                    sleep(250);

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
                    robot.strafeRight(500, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.forward(1800,1);
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

                    robot.backward(2000,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeLeft(5500, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    //drop off skystone
                    robot.setUpMotors();
                    robot.turnLeft(1800,1);
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

                    robot.backward(2800,1);
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

                    robot.backward(1000,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeRight(1100, 1);
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

                    robot.forward(2500,1);
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

                    robot.backward(2700,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeLeft(6200, 1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.setUpMotors();
                    robot.turnLeft(1800,1);
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