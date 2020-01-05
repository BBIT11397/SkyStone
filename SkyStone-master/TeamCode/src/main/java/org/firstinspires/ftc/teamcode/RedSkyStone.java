
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
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

        robot.forward(600, 0.25);
        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "forward to stones");
            telemetry.update();
            idle();
        }
        robot.allMotorsStop();

        sleep(250);

        double firstBlock = robot.colorSensorRight.alpha();
        telemetry.addLine()
                .addData("Right", firstBlock);
        telemetry.update();

        double secondBlock = robot.colorSensorLeft.alpha();
        telemetry.addLine()
                .addData("Left", secondBlock);
        telemetry.update();

        boolean foundBlock = false;

        //decide what block is skystone
        if(firstBlock <= secondBlock && opModeIsActive()){

            if(firstBlock + 10000 < secondBlock && opModeIsActive()) {
                //found skystone
                foundBlock = true;
                //back to first block

                telemetry.addData("first", "first");
                telemetry.update();

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

                robot.backward(2400,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "reline");
                    idle();
                }
                robot.allMotorsStop();
                sleep(250);

                //over line
                robot.strafeRight(4500, 1);
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

            } else {

                telemetry.addData("first", "third");
                telemetry.update();

                robot.backward(600,1);
                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "backward");
                    telemetry.update();
                    idle();
                }
                robot.allMotorsStop();
                sleep(500);

                robot.strafeLeft(1100, 0.5);
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

                robot.backward(2500,1);
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

                robot.setUpMotors();
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

                    telemetry.addData("second", "second");
                    telemetry.update();

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
                    sleep(250);

                    robot.setUpMotors();
                    robot.strafeLeft(500, 1);
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

                    robot.backward(2400,1);
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

                    telemetry.addData("second", "third");
                    telemetry.update();

                    robot.backward(1000,1);
                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "reline");
                        idle();
                    }
                    robot.allMotorsStop();
                    sleep(250);

                    robot.strafeLeft(1100, 1);
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

                    robot.setUpMotors();
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