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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;





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

public class UltimateManejando extends LinearOpMode {
    static private DcMotor backLeftMotor;
    static private DcMotor backRightMotor;
    static private Blinker control_Hub;
    static private Blinker expansion_Hub_2;
    static private DcMotor frontLeftMotor;
    static private DcMotor frontRightMotor;
    static private IMU imu;
    private HardwareDevice webcam_1;
    static private Orientation angles;
    static private double velocityofcar = 0.8;
    //static private DcMotor base_giratoria;
    //static private DcMotor elevador;
    static private Servo brazo_d;
    static private Servo brazo_i;
    static private Servo pinza;
    static private DcMotor basemotor;
    static private DcMotor corehex_d;
    static private DcMotor corehex_i;
    static private ElapsedTime runtime = new ElapsedTime();
    


    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        pinza = hardwareMap.get(Servo.class, "pinza");
        brazo_d = hardwareMap.get(Servo.class, "brazo_d");
        brazo_i = hardwareMap.get(Servo.class, "brazo_i");
        corehex_d = hardwareMap.get(DcMotor.class, "corehex_d");
        corehex_i = hardwareMap.get(DcMotor.class, "corehex_i");
        basemotor = hardwareMap.get(DcMotor.class, "basemotor");
        //elevador = hardwareMap.get(DcMotor.class, "elevador");
        //base_giratoria = hardwareMap.get(DcMotor.class, "base_giratoria");
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addData("Status", "Initialized");
        
        
        telemetry.update();
        imu.resetYaw();
        double tltPowery = 0;
        double trtPowery = 0;
        double tltPowerx = 0;
        double trtPowerx = 0;
        float tltLevery = 0;
        float trtLevery = 0;
        float tltLeverx = 0;
        float trtLeverx = 0;
        boolean button_a1 = false;
        boolean button_b1 = false;
        boolean button_y1 = false;
        boolean button_x1 = false;
        boolean button_a2 = false;
        boolean button_b2 = false;
        boolean button_y2 = false;
        float right_trigger1 = 0;
        float left_trigger1 = 0;
        float right_trigger2 = 0;
        float left_trigger2 = 0;
        boolean right_bumper1 = false;
        boolean left_bumper1 = false;
        boolean right_bumper2 = false;
        boolean left_bumper2 = false;
        double direction = 0;
        int pon = 1;
        double correctdir = 0;
        double absolutey = 0;
        double absolutex = 0;
        double radianrobotdir = 0;
        int valor_elevador = 0;
        int valor_pinza = 0;
        int velocidad = 2;
        char modo = 'Y';
        double triggers1 = 0.0;
        double triggers2 = 0.0;
        
        brazo_d.setDirection(Servo.Direction.REVERSE);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        corehex_d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corehex_d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        corehex_i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corehex_i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        basemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        basemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tltPowery = -this.gamepad1.left_stick_y;
            trtPowery = -this.gamepad1.right_stick_y;
            tltPowerx = -this.gamepad1.left_stick_x;
            trtPowerx = -this.gamepad1.right_stick_x;
            tltLevery = -this.gamepad2.left_stick_y;
            trtLevery = -this.gamepad2.right_stick_y;
            tltLeverx = -this.gamepad2.left_stick_x;
            trtLeverx = -this.gamepad2.right_stick_x;
            
            button_y1 = this.gamepad1.y;
            button_b1 = this.gamepad1.b;
            button_a1 = this.gamepad1.a;
            button_x1 = this.gamepad1.x;
            
            if (button_a1 == true){
                modo = 'A';
            }else if (button_b1 == true){
                modo = 'B';
            }else if (button_y1 == true){
                modo = 'Y';
            } else if (button_x1 == true){
                modo = 'X';
            }
            
            button_y2 = this.gamepad2.y;
            button_b2 = this.gamepad2.b;
            button_a2 = this.gamepad2.a;
            
            if(button_b2 == true){
                valor_pinza = 1;
            }else if (button_b2 == false){
                valor_pinza = 0;
            }
            
            right_trigger1 = this.gamepad1.right_trigger;
            left_trigger1 = this.gamepad1.left_trigger;
            
            triggers1 = (Math.abs(triggers1-0.5) <= 0.5) ? triggers1 + 0.01*(right_trigger1 - left_trigger1) : triggers1;
            if(triggers1 > 1){
                triggers1=1;
            }else if(triggers1 < 0){
                triggers1=0;
            }
            
            right_trigger2 = this.gamepad2.right_trigger;
            left_trigger2 = this.gamepad2.left_trigger;
            
            triggers2 = (Math.abs(triggers2-0.5) <= 0.5) ? triggers2 + 0.03*(right_trigger2 - left_trigger2) : triggers2;
            if(triggers2 > 0.85){
                triggers2=0.85;
            }else if(triggers2 < 0){
                triggers2=0;
                
            }
            telemetry.addData("Min position", brazo_i.MIN_POSITION);
            telemetry.addData("Direction", brazo_i.getDirection());
            right_bumper1 = this.gamepad1.right_bumper;
            left_bumper1 = this.gamepad1.left_bumper;
            
            if(runtime.seconds() >= 1 && right_bumper1 == true){
                velocityofcar = velocityofcar + 0.2;
                runtime.reset();
            }else if (runtime.seconds() >= 1 && left_bumper1 == true){
                velocityofcar = velocityofcar - 0.2;
                runtime.reset();
            }
            
            right_bumper2 = this.gamepad2.right_bumper;
            left_bumper2 = this.gamepad2.left_bumper;
            
            if(right_bumper2==true){
             
             valor_elevador = valor_elevador+1;   
             
            }else if(left_bumper2==true){
                
             valor_elevador = valor_elevador-1;
                
            }
            
            if(valor_elevador > 1){
                valor_elevador = 1;
            }else if(valor_elevador < 0){
                valor_elevador = 0;
            }
            
            
            telemetry.addData("Status", "Running");
            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX , AngleUnit.DEGREES);
            pon = ((trtPowerx >= 0) ? 1 : -1);
            direction = Math.acos((trtPowery)/(Math.sqrt(trtPowerx*trtPowerx + trtPowery*trtPowery))) * (180/Math.PI);
            correctdir = ((pon*direction > -182) ? pon*direction : correctdir);
            radianrobotdir  = angles.firstAngle*(Math.PI/180);
            absolutey = Math.cos(radianrobotdir)*tltPowery + Math.sin(radianrobotdir)*tltPowerx;
            absolutex = -Math.sin(radianrobotdir)*tltPowery + Math.cos(radianrobotdir)*tltPowerx;
            telemetry.addData("YAWZ", angles.firstAngle);
            telemetry.addData("direction", correctdir);
            telemetry.addData("triggers2", triggers2);
            
            if(modo == 'A'){
                mecanum(absolutey, -absolutex, trtPowery, getRightDirection(correctdir , angles.firstAngle));
            }else if(modo == 'B'){
                easyDrive(tltPowery, tltPowerx, trtPowery, -trtPowerx);
            }else if(modo == 'Y'){
                mecanum(tltPowery, -tltPowerx, trtPowery, -trtPowerx);
            }else if(modo == 'X'){
                
            }
            
            pinza_function((button_b2 == true) ? 1 : 0.42f);
            elevador(tltLevery);
            brazo_function(triggers2);
            base_function(triggers1);
            telemetry.addData("velocidad", velocityofcar);
            telemetry.addData("brazo_I", brazo_i.getPosition());
            telemetry.addData("brazo_D", brazo_d.getPosition());
            telemetry.addData("Front Left Motor", frontLeftMotor.getCurrentPosition()/560f);
            telemetry.addData("Front Right Motor", frontRightMotor.getCurrentPosition()/560f);
            telemetry.addData("Back Left Motor", backLeftMotor.getCurrentPosition()/560f);
            telemetry.addData("Back Right Motor", backRightMotor.getCurrentPosition()/560f);
            telemetry.addData("corehex_i", corehex_i.getCurrentPosition()/288f);
            telemetry.addData("corehex_d", corehex_d.getCurrentPosition()/288f);
            telemetry.addData("triggers1", triggers1);
            telemetry.addData("BaseMotor", basemotor.getCurrentPosition()/560f);
            telemetry.update();

        }
    }
    
    static void mecanum(double ly, double lx, double ry, double rx){
        frontLeftMotor.setPower(velocityofcar*(ly+lx-rx));
        backLeftMotor.setPower(velocityofcar*(ly-lx-rx));
        frontRightMotor.setPower(velocityofcar*(ly-lx+rx));
        backRightMotor.setPower(velocityofcar*(ly+lx+rx));
    }
    
    static void easyDrive(double ly, double lx, double ry, double rx){
        if (ly != 0 ){
            frontLeftMotor.setPower(velocityofcar*ly);
            backLeftMotor.setPower(velocityofcar*ly);
            frontRightMotor.setPower(velocityofcar*ly);
            backRightMotor.setPower(velocityofcar*ly);
        } else {
            frontLeftMotor.setPower(velocityofcar*-rx);
            backLeftMotor.setPower(velocityofcar*-rx);
            frontRightMotor.setPower(velocityofcar*rx);
            backRightMotor.setPower(velocityofcar*rx);
        }
    }
    
    static void base_function(double valor0){
        if(basemotor.getCurrentPosition()/560f > valor0+0.1){
            basemotor.setPower(-0.3);
        }else if(basemotor.getCurrentPosition()/560f < valor0-0.1){
            basemotor.setPower(0.3);
        }else{
            basemotor.setPower(0);
        }
    }
    
    static void elevador(float valor1){
        if(corehex_i.getCurrentPosition()/288f >= 5.5){
            valor1 = (valor1 > 0) ? 0 : valor1;
        }else if(corehex_i.getCurrentPosition()/288f <= 0){
            valor1 = (valor1 < 0) ? 0 : valor1;
        }
        corehex_i.setPower(valor1);
        corehex_d.setPower(-valor1);
    }
    
    static void pinza_function(float valor2){
     pinza.setPosition(valor2);
    }
    
    static void brazo_function (double valor3){
        brazo_d.setPosition(valor3);
        brazo_i.setPosition(valor3);
    }
    

    
    static double getRightDirection(double dir, double robotdir){
        int pin = ((robotdir>=0) ? 1 : -1);
        if(Math.abs(dir) > 180 - 1.5 && Math.abs(dir) < 180 + 1.5){
            if(Math.abs(robotdir) < 180 - 1.5){
                return -0.4*pin;
            }else{
                return 0;
            }
        }else if(robotdir < dir - 1.5){
            return -0.4;
        } else if (robotdir > dir + 1.5){
            return 0.4;
        } else {
            return 0;
        }
    }
}
