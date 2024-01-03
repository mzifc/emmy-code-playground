package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//@Disabled

// created by ME Zifchak - updated 7/7/23 15:59:56

public class Microwave_TeleOp extends LinearOpMode {
    // ex version has velocity measurements
    DcMotorEx motor, slidesLeft, slidesRight;
    
    PIDController leftController = new PIDController(0.25, 0, 0);
    PIDController rightController = new PIDController(0.25, 0, 0);
    
    @Override
    public void runOpMode() throws InterruptedException {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesRight = hardwareMap.get(DcMotorEx.class, "slidesRight");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        clawWrist = hardwareMap.servo.get("clawWrist");
        clawGripper = hardwareMap.servo.get("clawGripper");

        // reversing right motor!
        slidesRight.setDirection(DcMotorEx.Direction.REVERSE);
        
        // uses breaking to slow the motor down faster
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // disables velocity control but not encoder from counter
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        // position in tick cxwedw3s where we want the motor to run to
        int targetPosition = 0;

        int armGround = 0;
        int slideGround = 0;
        int armLow = 1;
        int armMid = 2;
        int armHigh = 3;
        int slidesHigh = 3;
        
        // loop that runs while the program is running
        while (opModeIsActive()) {

            /* thomas aperture example
            while(opmodeisactive()) {
                double slideMotorTargetPos = ...;
                if(buttonPressed) {
                    anglerServo.setPosition(x);
                    armServo.setPosition(y);
                    slideMotorTargetPos = z;
                }
                double power = slideMotorPID.run(slideMotor.getCurrentPosition-slideMotorTargetPos)
                slideMotor.setPower(power);
            }
            */

            // (x) pick up position
            if (gamepad2.x){ 
                targetPosition = GROUND_POSITION;
                    armRight.setPosition(x);
                    armLeft.setPosition(x);
                clawWrist.setPosition(y);
                clawGripper.setPosition(0.2); // open position
            }
            // (a) low scoring position
            else if (gamepad2.a){ 
                targetPosition = LOW_POSITION;
                    armRight.setPosition(x);
                    armLeft.setPosition(x);
                clawWrist.setPosition(y);
                clawGripper.setPosition(0.2); // open position
            }
            // (b) medium scoring position
            else if (gamepad2.b){ 
                targetPosition = MID_POSITION;
                    armRight.setPosition(x);
                    armLeft.setPosition(x);
                clawWrist.setPosition(y);
                clawGripper.setPosition(0.2); // open position
            }
            // (y) high scoring position
            else if (gamepad2.y){ 
                targetPosition = LOW_POSITION;
                    armRight.setPosition(x);
                    armLeft.setPosition(x);
                clawWrist.setPosition(y);
                clawGripper.setPosition(0.2); // open position
            }

            // (right bumper) hanging position
            if (gamepad2.right_bumper) {
               targetPosition = GROUND_POSITION; 
            }
            // (dpad right) manual open claw
            if (gamepad2.dpad_right) {
                clawGripper.setPosition(0.2); // open
            }
            // (dpad left) manual close claw
            else if (gamepad2.dpad_left) {
                clawGripper.setPosition(1); // closed
            }
            // (dpad up) manual move wrist up
            else if (gamepad2.dpad_up) {
                clawWrist.setPower(0.5); 
            }
            // (dpad down) manual move wrist down
                clawWrist.setPower(-0.5); 
            }
            else {
                clawWrist.setPower(0); }
            
            // silly slides telemetry
            telemetry.addData("Left Slides", slidesLeft.getCurrentPosition());
            telemetry.addData("Right Slides", slidesRight.getCurrentPosition());
            
            // silly target position telemetry
            telemetry.addData("Target Position", targetPosition);

            // update PID controller
            double leftPower = leftController.update(targetPosition, slidesLeft.getCurrentPosition());
            double rightPower = rightController.update(targetPosition, slidesRight.getCurrentPosition());
            
            // silly power telemetry + updating
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update(); 
            
            // assign motor the PID output
            slidesLeft.setPower(leftPower);  
            slidesRight.setPower(rightPower); 
            
        }
    }

}
