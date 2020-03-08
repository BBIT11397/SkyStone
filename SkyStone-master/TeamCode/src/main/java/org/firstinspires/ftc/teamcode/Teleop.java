/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;


import android.nfc.cardemulation.OffHostApduService;
import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="bbit")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();

    // could also use HardwarePushbotMatrix class.
    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double strafeLeft;
        double strafeRight;
        boolean conveyerOn = false;
        double conveyerDirection;
        double currentLevelPosition;
        double currentLSPosition;
        int currentARMPosition;
        int armLeadLevelPosition = 0;
        int armMotorLevelPosition = 0 ;
        int newARMPositon = 0;
        double swingIn = 0.4;

        double powerDown = 1;
        double powerUp = -1;

        double FORWARD = 1;
        double BACKWARDS = -1;
        boolean ON = true;
        boolean OFF = false;

        boolean swingArmMoving = false;
        boolean grabberDown = false;
        boolean leftTriggerPressed = false;
        boolean rightTriggerPressed = false;
        boolean capstoneOff = false;

        int armLevel;

        ElapsedTime x2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        x2Timer.reset();

        ElapsedTime y2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        y2Timer.reset();

        ElapsedTime a2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        a2Timer.reset();

        ElapsedTime dpad2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        dpad2Timer.reset();

        ElapsedTime x1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        x1Timer.reset();

        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;

        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

   //     robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grabberDown = false;

        capstoneOff = false;

        armLevel = -1;

        waitForStart();

        robot.swing.setPosition(swingIn);

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            turn = -gamepad1.right_stick_x;

            strafeLeft = gamepad1.left_trigger;
            strafeRight = gamepad1.right_trigger;

            // Combine drive and turn for blended motion.
            left = drive - turn;
            right = drive + turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            if(gamepad2.b){
                left /= 4;
                right /= 4;
                strafeLeft /= 4;
                strafeRight /= 4;
            }

            if (strafeLeft != 0 || strafeRight != 0) {
                if (strafeLeft != 0) {
                    robot.rightBack.setPower(-strafeLeft);
                    robot.rightFront.setPower(strafeLeft);
                    robot.leftFront.setPower(-strafeLeft);
                    robot.leftBack.setPower(strafeLeft);
                }

                if (strafeRight != 0) {
                    robot.leftFront.setPower(strafeRight);
                    robot.leftBack.setPower(-strafeRight);
                    robot.rightBack.setPower(strafeRight);
                    robot.rightFront.setPower(-strafeRight);
                }
            } else {
                robot.leftBack.setPower(left);
                robot.rightBack.setPower(right);
                robot.leftFront.setPower(left);
                robot.rightFront.setPower(right);
            }


            if (gamepad1.y) {
                robot.jaw.setPosition(0.7);
            }

            if (gamepad1.a) {
                robot.jaw.setPosition(0.1);
            }

            if (gamepad2.dpad_up && dpad2Timer.milliseconds() > 500) {
                if (armLevel < 3) {
                    armLevel += 1;
                }
                dpad2Timer.reset();
            }

            if (gamepad2.dpad_down && dpad2Timer.milliseconds() > 500) {
                if (armLevel > 0){
                    armLevel -= 1;
                }
                dpad2Timer.reset();
            }

                //leadscrew
            if (rightTriggerPressed == true|| leftTriggerPressed == true || armLevel <= 3 || armLevel >= 0) {
                if (rightTriggerPressed == true|| leftTriggerPressed == true ) {
                    armLevel = -1;
                    robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (rightTriggerPressed == true) {
                        currentLSPosition = robot.leadScrew.getCurrentPosition();
                        if (currentLSPosition <= 12900) {
                            robot.leadScrew.setPower(1);
                        } else {
                            robot.leadScrew.setPower(0);
                        }
                    }
                    if (leftTriggerPressed == true) {
                        currentLSPosition = robot.leadScrew.getCurrentPosition();
                        if (currentLSPosition >= 300) {
                            robot.leadScrew.setPower(-1);
                        } else {
                            robot.leadScrew.setPower(0);
                        }
                    }
                } else {
                    if (armLevel == -1){
                        robot.leadScrew.setPower(0);
                    }
                }
                /*

                if (armLevel >= 0){
                    armLeadLevelPosition = robot.leadScrew.getCurrentPosition();
                    if(armLeadLevelPosition <= 3263 && armLeadLevelPosition >= 2863){
                        robot.leadScrew.setPower(0);
                    } else {
                        robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (robot.leadScrew.getTargetPosition() != 3063){
                            robot.leadScrew.setTargetPosition(3063);
                            robot.leadScrew.setPower(1);
                        }

                    }
                }

                 */
            } else {
                robot.leadScrew.setPower(0);
            }

            //ARM MOTOR
            if (gamepad1.dpad_down || gamepad1.dpad_up || armLevel <= 3|| armLevel >= 0) {
                if (gamepad1.dpad_down || gamepad1.dpad_up ) {
                    armLevel = -1;
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (gamepad1.dpad_down) {
                        currentARMPosition = robot.armMotor.getCurrentPosition();
                        if (currentARMPosition <= 2100) {
                            robot.armMotor.setPower(powerDown);
                        } else {
                            robot.armMotor.setPower(0);
                        }
                    }
                    if (gamepad1.dpad_up) {
                        currentARMPosition = robot.armMotor.getCurrentPosition();
                        if (currentARMPosition >= -6000) {
                            robot.armMotor.setPower(powerUp);
                        } else {
                            robot.armMotor.setPower(0);
                        }
                    }
                } else {
                    if (armLevel == -1) {
                        robot.armMotor.setPower(0);
                    }
                }

                /*
                if (armLevel == 0){
                    armMotorLevelPosition = robot.armMotor.getCurrentPosition();
                    if(armMotorLevelPosition >= 1900 && armMotorLevelPosition <= 2300){
                        robot.armMotor.setPower(0);
                    } else {
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if(robot.armMotor.getTargetPosition()!= 2100){
                            robot.armMotor.setTargetPosition(2100);
                            robot.armMotor.setPower(1);
                        }
                    }
                }

                 */
                /*
                if (armLevel == 1){
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setTargetPosition();
                }
                if (armLevel == 2){
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setTargetPosition();
                }
                if (armLevel == 3){
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setTargetPosition();
                }

                 */

            } else {
                robot.armMotor.setPower(0);
            }


            /*
            if (gamepad1.b) {
                robot.paddleTop.setPosition(0);
            }

            if (gamepad1.x) {
                robot.paddleTop.setPosition(0.8) ;
            }

             */

            //SWING
            if (gamepad2.right_bumper) {
                robot.swing.setPosition(0.85);
            }
            //SWING
            if (gamepad2.left_bumper) {
                robot.swing.setPosition(swingIn);
            }

            if (gamepad2.left_trigger > 0){
                leftTriggerPressed = true;
            } else {
                leftTriggerPressed = false;
            }

            if (gamepad2.right_trigger > 0){
                rightTriggerPressed = true;
            } else {
                rightTriggerPressed = false;
            }

            if (gamepad2.y && y2Timer.milliseconds() > 500){
                if(grabberDown) {
                    robot.foundationGrabberR.setPosition(0);
                    robot.foundationGrabberL.setPosition(1);
                    grabberDown = false;
                } else {
                //if (grabberDown == false){
                    robot.foundationGrabberR.setPosition(.55);
                    robot.foundationGrabberL.setPosition(.4);
                    grabberDown = true;
                }
                y2Timer.reset();
            }

            if (gamepad2.x && x2Timer.milliseconds() > 500){
                if(grabberDown) {
                    robot.foundationGrabberL.setPosition(1);
                    grabberDown = false;
                } else {
                    //if (grabberDown == false){
                    robot.foundationGrabberL.setPosition(.4);
                    grabberDown = true;
                }
                x2Timer.reset();
            }

            if (gamepad2.a && a2Timer.milliseconds() > 500){
                if(grabberDown) {
                    robot.foundationGrabberR.setPosition(0);
                    grabberDown = false;
                } else {
                    //if (grabberDown == false){
                    robot.foundationGrabberR.setPosition(.55);
                    grabberDown = true;
                }
                a2Timer.reset();
            }

            if (gamepad1.x && x1Timer.milliseconds() > 500){
                if(capstoneOff) {
                    robot.capstoneDropper.setPosition(1);
                    capstoneOff = false;
                } else {
                    robot.capstoneDropper.setPosition(0.15);
                    capstoneOff = true;
                }
                x1Timer.reset();
            }


            if(gamepad1.b){
                robot.stoneHand.setPosition(0);
                robot.stoneFinger.setPosition(0);            }

            if (gamepad2.right_stick_y > 0.5 || gamepad2.right_stick_y < -0.5){
                    robot.tapeMeasure.setPower(gamepad2.right_stick_y);
            } else {
                robot.tapeMeasure.setPower(0);
            }

            telemetry.addLine()
                    .addData("armLeadLevelPosition", armLeadLevelPosition);
            telemetry.addLine()
                    .addData("timer", dpad2Timer.milliseconds());
            telemetry.addLine()
                    .addData("arm motor", robot.armMotor.getCurrentPosition());
            telemetry.addLine()
                    .addData("lead Screw", robot.leadScrew.getCurrentPosition());
            telemetry.addLine()
                    .addData("arm Level", armLevel);
            telemetry.update();


/*
            if (gamepad2.x) {
                swingArmMoving= true;
                robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leadScrew.setTargetPosition(4000);
                robot.leadScrew.setPower(1);
            }

            if (robot.leadScrew.getCurrentPosition() >= 4000 && swingArmMoving == true){
                robot.leadScrew.setPower(0);

                robot.swing.setPosition(.7);

                robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                swingArmMoving = false;
            }


 */
            /*
            if (gamepad1.b) {
                robot.handDown();
                sleep(250);

                //strafe left
                robot.rightBack.setPower(1);
                robot.rightFront.setPower(-1);
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(-1);
                sleep(250);

                robot.fingerGrab();
            }


            if (gamepad1.x) {
                robot.handUpWithStone();
                sleep(250);

                //strafe left
                //add encoder ticks
                robot.rightBack.setPower(1);
                robot.rightFront.setPower(-1);
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(-1);
                sleep(250);

                robot.handDownWithStone();
                sleep(250);

                robot.fingerRelease();
                sleep(250);

                robot.stoneStart();
            }

             */
/*
            if (gamepad1.b){
                robot.stoneHand.setPosition(.2);
            }

            if (gamepad1.x){
                robot.stoneHand.setPosition(.7);
            }

            if (gamepad1.right_bumper){
                robot.stoneFinger.setPosition(.425);
            }

            if (gamepad1.left_bumper){
                robot.stoneFinger.setPosition(1);
            }
*/

            /*
            if (gamepad1.right_bumper){
                            robot.blockStopperR.setPosition(0);
                            robot.blocksStopperL.setPosition(1);
                        }
            if (gamepad1.left_bumper){
                            robot.blockStopperR.setPosition(1);
                            robot.blocksStopperL.setPosition(0);
                        }




                //  CHANGE TO GP 2?
                if (myTimer.milliseconds() > 250) {
                    if (gamepad2.x || gamepad2.b) {
                        if (conveyerOn == ON) {
                            conveyerOn = OFF;
                        } else {
                            conveyerOn = ON;
                        }
                        if (conveyerOn && gamepad2.x) {
                            robot.beltMotor.setPower(FORWARD);
                 //           robot.sweeper.setPower(FORWARD);
                        }
                        if (conveyerOn && gamepad2.b) {
                            robot.beltMotor.setPower(BACKWARDS);
                   //         robot.sweeper.setPower(BACKWARDS);
                        }
                        if ((conveyerOn == OFF)) {
                            robot.beltMotor.setPower(0);
                     //       robot.sweeper.setPower(0);
                        }
                        myTimer.reset();
                    }
                }
                if (gamepad1.right_bumper) {
                    robot.paddleRight.setPosition(Servo.MAX_POSITION);
                } else {
                    robot.paddleRight.setPosition(0.3);
                }

                //LEVELER
                if (gamepad2.y) {
                    robot.leveler.setPosition(0.4);
                }

                if (gamepad2.a) {
                    robot.leveler.setPosition(0);
                }
*/
            }
        }
    }