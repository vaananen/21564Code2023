/*
Copyright 2023 FIRST Tech Challenge Team 21564

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class TestCode2023 extends LinearOpMode {
    
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private DcMotor armExtenderMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor pivotMotor;
    private Servo pixelServo;
    private Servo planeServo;
    private DcMotor spatulaMotor;
    private HardwareDevice webcam;
    private TouchSensor pushButton3;

    @Override
    public void runOpMode() {
        
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        
        
        //EXPANSION HUB (DRIVE) //
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        
        pixelServo = hardwareMap.get(Servo.class, "pixelServo");
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        
        
        //CONTROLLER HUB//
        spatulaMotor = hardwareMap.get(DcMotor.class, "spatulaMotor");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        armExtenderMotor = hardwareMap.get(DcMotor.class, "armExtenderMotor");
        
        webcam = hardwareMap.get(HardwareDevice.class, "webcam");
        
        
        TouchSensor pushButton3 = hardwareMap.get(TouchSensor.class, "pushButton3");
        
        
        //VARIABLES//
        
        boolean lastUp = false;
        
        double BRAKESPEED = 0.25;
        double spatulaSpeed = 1;
        
        double originalPixelPosition = 0;
        double originalPlanePosition = 0.75;
        
        double desiredPixelPosition = .55;
        double desiredPlanePosition = 0.3;
        
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;
        
        //REVERSE//
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (double)(y + x + rx) / denominator;
            backLeftPower = (double)(y - x + rx) / denominator;
            frontRightPower = (double)(y - x - rx) / denominator;
            backRightPower = (double)(y + x - rx) / denominator;
            
            frontRightMotor.setPower((double)frontRightPower);
            frontLeftMotor.setPower((double)frontLeftPower);
            backRightMotor.setPower((double)backRightPower);
            backLeftMotor.setPower((double)backLeftPower);
            
            //SPATULACODE//
            
            if(gamepad2.a){
                
                spatulaMotor.setPower(spatulaSpeed);
                
            }
            else if(gamepad2.b){
                
                spatulaMotor.setPower(-spatulaSpeed);
                
            }
            else{
                spatulaMotor.setPower(0);
            }
            
            //EXTENDERCODE//
            
            if(gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y < -0.05){
                
                armExtenderMotor.setPower((float)-gamepad2.left_stick_y);
                
            }
            else{
                armExtenderMotor.setPower(0);
            }
            
            
            //PIVOTCODE//
            
            if((gamepad2.right_stick_y > 0.05 || gamepad2.right_stick_y < -0.05) && !pushButton3.isPressed()){
                if(gamepad2.right_stick_y > 0.05){
                    lastUp = false;
                    pivotMotor.setPower(gamepad2.right_stick_y/2);
                }
                else if(gamepad2.right_stick_y < -0.05){
                    lastUp = true;
                    pivotMotor.setPower(gamepad2.right_stick_y/1.5);
                }
                
            }
            else if(lastUp && !pushButton3.isPressed()){
                pivotMotor.setPower(-BRAKESPEED);
            }
            else if(!pushButton3.isPressed()){
                pivotMotor.setPower(-.1);
            }
            else{
                pivotMotor.setPower(0)
;            }
            
            //SERVOS//
            
            boolean planeOn = false;
            boolean pixelOn = false;
            
            if(gamepad2.x){
                
                pixelServo.setPosition(desiredPixelPosition);
                
            }
            else{
                pixelServo.setPosition(originalPixelPosition);
            }
            
            if(gamepad2.y){
                
                planeServo.setPosition(desiredPlanePosition);
                
            }
            else{
                planeServo.setPosition(originalPlanePosition);
            }
            
            telemetry.addData("pixelServo", pixelServo.getPosition());
            telemetry.addData("planeServo", planeServo.getPosition());
            
            telemetry.addData("pushButton3", pushButton3.isPressed());
            telemetry.addData("pixelOn", pixelOn);
            telemetry.addData("planeOn", planeOn);
        
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
