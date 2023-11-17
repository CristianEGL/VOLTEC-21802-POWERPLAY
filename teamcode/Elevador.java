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

public class Elevador extends LinearOpMode {
    private Blinker control_Hub;
    static private DcMotor corehex;
    private IMU imu;
    static private Servo brazo_d;
    static private Servo brazo_i;
    static private Servo pinza;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        corehex = hardwareMap.get(DcMotor.class, "corehex");
        imu = hardwareMap.get(IMU.class, "imu");
        pinza = hardwareMap.get(Servo.class, "pinza");
        brazo_d = hardwareMap.get(Servo.class, "brazo_d");
        brazo_i = hardwareMap.get(Servo.class, "brazo_i");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        corehex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corehex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float rotation = 0;
        float tltPowery = 0;
        boolean button_b1 = false;
        float right_trigger = 0;
        float left_trigger = 0;
        int valor_pinza = 0;
        double triggers = 0.0;
        brazo_d.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tltPowery = -this.gamepad1.left_stick_y;
            button_b1 = this.gamepad1.b;
            
             if(button_b1 == true){
                valor_pinza = 1;
            }else if (button_b1 == false){
                valor_pinza = 0;
            }
            
            right_trigger = this.gamepad1.right_trigger;
            left_trigger = this.gamepad1.left_trigger;
            
            triggers = (Math.abs(triggers-0.5) <= 0.5) ? triggers + 0.01*(right_trigger - left_trigger) : triggers;
            if(triggers > 1){
                triggers=1;
            }else if(triggers < 0){
                triggers=0;
            }
            
            telemetry.addData("Status", "Running");
            corehex.setPower(tltPowery);
            pinza_function((button_b1 == true) ? 1 : 0.42f);
            brazo_function(triggers);
            rotation = corehex.getCurrentPosition()/288;
            telemetry.addData("Elevador", rotation);
            telemetry.update();

        }
        
    }
    
    static void pinza_function(float valor2){
     pinza.setPosition(valor2);
    }
    
    static void brazo_function (double valor3){
        brazo_d.setPosition(valor3);
        brazo_i.setPosition(valor3);
    }
    
    static void Lifter(float altura, float rotation){
        if (altura > rotation){
            corehex.setPower(1);
            
        }else if (altura < rotation){
            corehex.setPower(-1);
            
        }
    }
}
