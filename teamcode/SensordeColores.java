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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
@Autonomous

public class SensordeColores extends LinearOpMode {
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private Blinker control_Hub;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private Servo servito;
    private ColorSensor colorsens;
    private IMU imu;


    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        servito = hardwareMap.get(Servo.class, "Servito");
        colorsens = hardwareMap.get(ColorSensor.class, "colorsens");
        imu = hardwareMap.get(IMU.class, "imu");
        float completo = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        colorsens.enableLed(true);
        

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            completo = colorsens.green() + colorsens.red() + colorsens.blue();
            telemetry.addData("Color verde ", colorsens.green()/completo);
            telemetry.addData("Color rojo", colorsens.red()/completo);
            telemetry.addData("Color azul", colorsens.blue()/completo);
            if(detectThing(colorsens.green()/completo, colorsens.red()/completo, colorsens.blue()/completo, 0.41f, 0.26f, 0.32f, "Nada") == true){
                telemetry.addData("Encontre", "Nada");
            }
            if(detectThing(colorsens.green()/completo, colorsens.red()/completo, colorsens.blue()/completo, 0.24f, 0.11f, 0.63f, "Cono Azul") == true){
                telemetry.addData("Encontre", "Cono Azul");
            }
            
            telemetry.update();
        }
        
        colorsens.enableLed(false);
        
    }
    
    static boolean detectThing(float verde, float rojo, float azul, float v, float r, float a, String cosa){
        if(verde <= v + 0.03 && verde >= v - 0.03 && rojo <= r + 0.03 && rojo >= r - 0.03 && azul <= a + 0.03 && azul >= a - 0.03){
            return true;
        }
        return false;
    }
    
    
}
