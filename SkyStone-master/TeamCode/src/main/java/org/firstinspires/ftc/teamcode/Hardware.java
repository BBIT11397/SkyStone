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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;


  //This is NOT an opmode.

public class Hardware {
    /* Public OpMode members. */
    public DcMotor leftFront        = null;
    public DcMotor leftBack         = null;
    public DcMotor rightFront       = null;
    public DcMotor rightBack        = null;
    public DcMotor armMotor         = null;
    public DcMotor leadScrew        = null;
    public DcMotor tapeMeasure      = null;

    public Servo jaw                = null;
    public Servo swing              = null;
    public Servo foundationGrabberR = null;
    public Servo foundationGrabberL = null;
    public Servo stoneHand          = null;
    public Servo stoneFinger        = null;

    public Servo paddleTop          = null;
    public Servo blockStopperR      = null;
    public Servo blocksStopperL     = null;
    public Servo capstoneDropper    = null;

    /* local OpMode members. */
    HardwareMap hwMap               =  null;
    private ElapsedTime period      = new ElapsedTime();
    private Telemetry telemetry;

    public ColorSensor colorSensorRight;
    public ColorSensor colorSensorLeft;

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init (HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        telemetry.addData("hardware init:" , "enter");
        telemetry.update();

        colorSensorRight = hwMap.get(ColorSensor.class, "colorSensorRight");
        colorSensorLeft = hwMap.get(ColorSensor.class,"colorSensorLeft");

        // Define and Initialize Motors
        leftBack    = hwMap.get(DcMotor.class, "leftBack");
        rightFront  = hwMap.get(DcMotor.class, "rightFront");
        leftFront   = hwMap.get(DcMotor.class, "leftFront");
        rightBack   = hwMap.get(DcMotor.class, "rightBack");

        armMotor    = hwMap.get(DcMotor.class, "armMotor");
        leadScrew   = hwMap.get(DcMotor.class,"leadScrew");
        tapeMeasure = hwMap.get(DcMotor.class,"tapeMeasure");

        jaw     = hwMap.get(Servo.class,"jaw");
        swing   = hwMap.get(Servo.class, "swing");
        foundationGrabberR = hwMap.get(Servo.class, "foundationGrabberR");
        foundationGrabberL = hwMap.get(Servo.class, "foundationGrabberL");
        stoneHand = hwMap.get(Servo.class, "stoneHand");
        stoneFinger = hwMap.get(Servo.class, "stoneFinger");
        capstoneDropper = hwMap.get(Servo.class, "capstoneDropper");


        //paddleTop   = hwMap.get(Servo.class,"paddleTop");
        //blockStopperR = hwMap.get(Servo.class, "blockStopperR");
        //blocksStopperL = hwMap.get(Servo.class, "blockStopperL");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        leadScrew.setDirection(DcMotor.Direction.FORWARD);
        tapeMeasure.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);

        armMotor.setPower(0);
        leadScrew.setPower(0);
        tapeMeasure.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tapeMeasure.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leadScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        swing.setPosition(0.2);
        jaw.setPosition(0.1);
        foundationGrabberR.setPosition(0);
        foundationGrabberL.setPosition(1);

        capstoneDropper.setPosition(1);

        //init stone hand and finger
        stoneStart();

        //paddleTop.setPosition(0.8);
        //blockStopperR.setPosition(0);
        //blocksStopperL.setPosition(.655);

        telemetry.addData("hardware init:" , "exit");
        telemetry.update();
    }

    public void setUpMotors() {
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void allMotorsStop (){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public boolean checkMotorIsBusy (){
        if (leftFront.isBusy()){
            return true;
        }
        return false;
    }

    public void forward(int position, double speed){
        setUpMotors();
        leftFront.setTargetPosition(-position);
        leftBack.setTargetPosition(-position);
        rightBack.setTargetPosition(-position);
        rightFront.setTargetPosition(-position);

        rightBack.setPower(speed);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void backward(int position, double speed){
        setUpMotors();
        leftFront.setTargetPosition(position);
        leftBack.setTargetPosition(position);
        rightBack.setTargetPosition(position);
        rightFront.setTargetPosition(position);

        rightBack.setPower(speed);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void strafeRight(int position, double speed){
        setUpMotors();
        leftFront.setTargetPosition(position);
        leftBack.setTargetPosition(-position);
        rightBack.setTargetPosition(position);
        rightFront.setTargetPosition(-position);

        rightBack.setPower(speed);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void strafeLeft(int position, double speed){
        setUpMotors();
        leftFront.setTargetPosition(-position);
        leftBack.setTargetPosition(position);
        rightBack.setTargetPosition(-position);
        rightFront.setTargetPosition(position);

        rightBack.setPower(speed);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void turnRight(int position, double speed){
        setUpMotors();
        leftFront.setTargetPosition(position);
        leftBack.setTargetPosition(position);
        rightBack.setTargetPosition(-position);
        rightFront.setTargetPosition(-position);

        rightBack.setPower(speed);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void turnLeft(int position, double speed){
        setUpMotors();
        leftFront.setTargetPosition(-position);
        leftBack.setTargetPosition(-position);
        rightBack.setTargetPosition(position);
        rightFront.setTargetPosition(position);

        rightBack.setPower(speed);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    public void blockGrabbberUP(){
        blockStopperR.setPosition(0);
        blocksStopperL.setPosition(1);
    }

    public void blockGrabberDOWN(){
        blockStopperR.setPosition(1);
        blocksStopperL.setPosition(0);
    }

    public void foundSkyStone(){
        backward(800,1);
        allMotorsStop();

        //open paddle top
        paddleTop.setPosition(0.655);

        forward(1800,1);
        allMotorsStop();

        //close paddle top
        paddleTop.setPosition(.2);

        backward(1000,1);
        allMotorsStop();
    }

    public void dropOffSkyStone(){
        forward(500,1);
        allMotorsStop();
        //open top paddle
        paddleTop.setPosition(0.2);
        backward(500,1);
        allMotorsStop();
    }

    public void stoneStart(){
        stoneHand.setPosition(.2);
        stoneFinger.setPosition(0);
    }

    public void handUp(){
        stoneHand.setPosition(0);
        stoneFinger.setPosition(0);
    }

    public void handDown() {
        stoneFinger.setPosition(.7);
        stoneHand.setPosition(.7);
    }

    public void handDownWithStone() {
        stoneHand.setPosition(.7);
        stoneFinger.setPosition(.425);
    }

    public void fingerGrab() {
        stoneFinger.setPosition(.3);
    }

    public void fingerRelease() {
        stoneFinger.setPosition(1);
    }

    public  void handUpWithStone() {
        stoneHand.setPosition(0);
        stoneFinger.setPosition(.425);
    }

    public void foundationGrabberDown() {
        foundationGrabberR.setPosition(.55);
        foundationGrabberL.setPosition(.4);
    }

    public void foundationGrabberUp() {
        foundationGrabberR.setPosition(0);
        foundationGrabberL.setPosition(1);
    }
}