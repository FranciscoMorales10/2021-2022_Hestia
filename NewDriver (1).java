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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="New Driver", group="Linear Opmode")
//@Disabled
public class NewDriver extends LinearOpMode {

    //========================================
    // DECLARE OPMODE MEMBERS
    //========================================

    // Misc
    private final ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    // We created a servo object an a variable to hold its position
    Servo ClawH;
    double ClawH_position = 0.0;
    Servo ClawF;
    double ClawF_position = 0.0;
    //Intake servos 
    Servo LeftServo;
    double LeftServo_position = 0.0;
    Servo RightServo;
    double RightServo_position = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //========================================
        // HARDWARE MAPPING
        //========================================

        // We have the hardware mapped to the servo object to the actual servo
        ClawH = hardwareMap.servo.get("ClawH");
        ClawF = hardwareMap.servo.get("ClawF");
        
        LeftServo = hardwareMap.servo.get("LeftServo");
        RightServo = hardwareMap.servo.get("RightServo");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double vertical = -gamepad1.left_stick_y;
            double horizontal  =  gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            frontRightPower = Range.clip((-vertical + horizontal) + turn, -1.0, 1.0);
            backRightPower = Range.clip((vertical - horizontal) + turn, -1.0, 1.0) ;
            frontLeftPower = Range.clip((-vertical - horizontal) - turn, -1.0, 1.0) ;
            backLeftPower = Range.clip((vertical + horizontal) - turn , -1.0, 1.0) ;

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            

            // A button to break instantly
            if (gamepad2.right_stick_button) {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
            }

            /*
             * Servo STUFF
             * */
                
                if (gamepad2.a) {
                    RightServo.setPosition(0.0);
                    LeftServo.setPosition(1.0);
                    
                } else if (gamepad2.b) {
                    LeftServo.setPosition(0.0);
                    RightServo.setPosition(1);
                }
                if(gamepad2.left_bumper){
                    LeftServo.setPosition(0.5);
                    RightServo.setPosition(0.5);
                } 
        
        
           // Both of them return and receive a value from 0.0 - 1.0
            ClawH.setPosition(gamepad2.right_trigger);
            ClawF.setPosition(gamepad2.left_trigger);


            /*
             * Telemetry
             * */

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
            telemetry.update();
        }
    }
}
