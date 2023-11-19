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

public class TestPID extends LinearOpMode {
    // ex version has velocity measurements
    DcMotorEx motor, slidesLeft, slidesRight;
    
    PIDController leftController = new PIDController(0.25, 0, 0);
    PIDController rightController = new PIDController(0.25, 0, 0);
    
    @Override
    public void runOpMode() throws InterruptedException {
        slidesLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesRight = hardwareMap.get(DcMotorEx.class, "slidesRight");

        // reversing right motor!
        slidesRight.setDirection(DcMotorEx.Direction.REVERSE);
        
        // uses breaking to slow the motor down faster
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // disables velocity control but not encoder from counter
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        // position in tick cxwedw3s where we want the motor to run to
        int targetPosition = 0;

        int GROUND_POSITION = 0;
        int HIGH_POSITION = 3;
        int MID_POSITION = 2;
        int LOW_POSITION = -50;
        
        // loop that runs while the program is running
        while (opModeIsActive()) {

            if (gamepad1.x){ targetPosition = GROUND_POSITION; }
            else if (gamepad1.y){ targetPosition = HIGH_POSITION;}
            else if (gamepad1.b){ targetPosition = MID_POSITION;}
            else if (gamepad1.a){ targetPosition = LOW_POSITION;}
            
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
