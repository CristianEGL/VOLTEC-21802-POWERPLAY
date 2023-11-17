/*
Copyright 2023 FIRST Tech Challenge Team FTC

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class CoreHexMotor extends LinearOpMode {
    private Blinker control_Hub;
    private Servo servito;
    private DcMotor corehex;
    private IMU imu;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        servito = hardwareMap.get(Servo.class, "Servito");
        corehex = hardwareMap.get(DcMotor.class, "corehex");
        imu = hardwareMap.get(IMU.class, "imu");
        float tltPowery = 0;
        float trtPowery = 0;
        float trtPowerx = 0;
        float rotation = 0;
        float ticks = 0;
        int pon = 1;
        double direction = 0;
        double correctdir = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        corehex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corehex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        waitForStart();
        
        
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tltPowery = -this.gamepad2.left_stick_y;
            trtPowery = -this.gamepad2.right_stick_y;
            trtPowerx = -this.gamepad2.right_stick_x;
            pon = ((trtPowerx >= 0) ? 1 : -1);
            direction = Math.acos((trtPowery)/(Math.sqrt(trtPowerx*trtPowerx + trtPowery*trtPowery))) * (180/Math.PI);
            correctdir = ((pon*direction > -182) ? pon*direction : correctdir);
            telemetry.addData("Status", "Running");
            // 288 ticks = 1 rotation
            ticks = corehex.getCurrentPosition();
            rotation = ticks/288;
            telemetry.addData("Encoder Hex", rotation);
            telemetry.addData("DEGREES", rotation*180);
            telemetry.addData("Direction", correctdir);
            corehex.setPower(tltPowery);
            
            if(rotation*180 > correctdir + 1){
                corehex.setPower(-0.3);
            }else if(rotation*180 < correctdir - 1){
                corehex.setPower(0.3);
            }else{
                corehex.setPower(0);
            }
            
            telemetry.update();

        }
    }
}
