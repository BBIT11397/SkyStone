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
        int newARMPositon = 0;

        double powerDown = -0.25;
        double powerUp = 0.5;

        double FORWARD = 1;
        double BACKWARDS = -1;
        boolean ON = true;
        boolean OFF = false;

        ElapsedTime myTimer = new ElapsedTime();

        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;

        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

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
                    robot.rightBack.setPower(strafeLeft);
                    robot.rightFront.setPower(-strafeLeft);
                    robot.leftFront.setPower(strafeLeft);
                    robot.leftBack.setPower(-strafeLeft);
                }

                if (strafeRight != 0) {
                    robot.leftFront.setPower(-strafeRight);
                    robot.leftBack.setPower(strafeRight);
                    robot.rightBack.setPower(-strafeRight);
                    robot.rightFront.setPower(strafeRight);
                }
            } else {
                robot.leftBack.setPower(left);
                robot.rightBack.setPower(right);
                robot.leftFront.setPower(left);
                robot.rightFront.setPower(right);
            }


            if (gamepad1.a) {
                robot.jaw.setPosition(0.4);
            }

            if (gamepad1.y) {
                robot.jaw.setPosition(0);
            }


            //ARM MOTOR
            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                //currentARMPosition = robot.armMotor.getCurrentPosition();
                if (gamepad1.dpad_down ) {
                    robot.armMotor.setPower(powerDown);
                }
                if (gamepad1.dpad_up) {
                    robot.armMotor.setPower(powerUp);
                }
            } else {
                robot.armMotor.setPower(0);
            }

            if (gamepad1.b) {
                robot.paddleTop.setPosition(0);
            }

            if (gamepad1.x) {
                robot.paddleTop.setPosition(0.8) ;
            }

            if (gamepad1.right_bumper){
                robot.blockStopperR.setPosition(0);
                robot.blocksStopperL.setPosition(1);
            }

            if (gamepad1.left_bumper){
                robot.blockStopperR.setPosition(1);
                robot.blocksStopperL.setPosition(0);
            }

            //SWING
            if (gamepad2.right_bumper) {
                robot.swing.setPosition(1);
            }
            //SWING
            if (gamepad2.left_bumper) {
                robot.swing.setPosition(0);
            }

            if (gamepad2.dpad_up || gamepad2.dpad_down) {
                if (gamepad2.dpad_up) {
                    currentLSPosition = robot.leadScrew.getCurrentPosition();
                    if (currentLSPosition <= 11600) {
                        robot.leadScrew.setPower(1);
                        telemetry.addLine()
                                .addData("lift arm", robot.leadScrew.getCurrentPosition());
                        telemetry.update();
                    } else {
                        robot.leadScrew.setPower(0);
                    }
                }
                if (gamepad2.dpad_down) {
                    currentLSPosition = robot.leadScrew.getCurrentPosition();
                    if (currentLSPosition >= 300){
                        robot.leadScrew.setPower(-1);
                        telemetry.addLine()
                                .addData("lift arm", robot.leadScrew.getCurrentPosition());
                        telemetry.update();
                    } else {
                        robot.leadScrew.setPower(0);
                    }
                }
            } else {
                robot.leadScrew.setPower(0);
                telemetry.addLine()
                        .addData("lift arm", robot.leadScrew.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad2.a){
                robot.foundationGrabberR.setPosition(0);
                robot.foundationGrabberL.setPosition(1);
            }

            if (gamepad2.y){
                robot.foundationGrabberR.setPosition(1);
                robot.foundationGrabberL.setPosition(0);
            }

/*
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


            if (gamepad1.left_bumper) {
                robot.paddleLeft.setPosition(0);
            }

            if (gamepad1.right_bumper){
                robot.paddleLeft.setPosition(0.7);
            }
*/
            }
        }
    }