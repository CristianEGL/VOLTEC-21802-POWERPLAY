/*
Copyright 2022 FIRST Tech Challenge Team FTC

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo.Direction;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class ProbandoServito extends LinearOpMode {
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private Blinker control_Hub;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private Servo brazo_d;
    private IMU imu;


    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        brazo_d = hardwareMap.get(Servo.class, "brazo_d");
        imu = hardwareMap.get(IMU.class, "imu");
        float buttonRT = 0;
        float buttonLT = 0;
        double tltPowery = 0;
        double revolution = 0;
        double oftick = 0;
        brazo_d.setDirection(Servo.Direction.REVERSE);
        
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tltPowery = -this.gamepad1.left_stick_y;
            buttonRT = this.gamepad1.right_trigger;
            buttonLT = this.gamepad1.left_trigger;
            
            telemetry.addData("Servo", brazo_d.getPosition());
            telemetry.addData("buttonRT", buttonRT);
            telemetry.addData("pinza", brazo_d.getDirection());
            // 8192 ticks per revolution
            oftick = frontRightMotor.getCurrentPosition();
            revolution = oftick/8192;
            telemetry.addData("encodersito", frontRightMotor.getCurrentPosition());
            telemetry.addData("rotaciones", revolution);
            //pinza.setPosition(buttonRT);
            
            telemetry.addData("MIN POSITION", brazo_d.MIN_POSITION);
            brazo_d.setPosition(buttonRT);
            
            
            telemetry.update();

        }
    }
}
