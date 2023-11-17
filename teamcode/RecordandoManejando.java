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
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
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

public class RecordandoManejando extends LinearOpMode {
    static private DcMotor backLeftMotor;
    static private DcMotor backRightMotor;
    static private Blinker control_Hub;
    static private DcMotor frontLeftMotor;
    static private DcMotor frontRightMotor;
    static private IMU imu;
    static private Orientation angles;
    static private double velocityofrotation = 0.4;
    static private double velocityofcar = 1;
    static private ElapsedTime runtime = new ElapsedTime();
    static private Servo brazo_d;
    static private Servo brazo_i;
    static private Servo pinza;
    
    


    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        pinza = hardwareMap.get(Servo.class, "pinza");
        brazo_d = hardwareMap.get(Servo.class, "brazo_d");
        brazo_i = hardwareMap.get(Servo.class, "brazo_i");
        
        brazo_d.setDirection(Servo.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        float tltPowery = 0;
        float trtPowery = 0;
        float tltPowerx = 0;
        float trtPowerx = 0;
        double direction = 0;
        double absolutey = 0;
        double absolutex = 0;
        boolean buttonRB = false;
        boolean buttonLB = false;
        float buttonRT = 0;
        float buttonLT = 0;
        boolean a = false;
        boolean b = false;
        boolean x = false;
        float altura = 0;
        int pon = 1;
        float correctdir = 0;
        double radianrobotdir = 0;
        imu.resetYaw();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tltPowery = (float) -this.gamepad1.left_stick_y;
            trtPowery = (float) -this.gamepad1.right_stick_y;
            tltPowerx = (float) -this.gamepad1.left_stick_x;
            trtPowerx = (float) -this.gamepad1.right_stick_x;
            buttonRB = this.gamepad1.right_bumper;
            buttonLB = this.gamepad1.left_bumper;
            buttonRT = this.gamepad1.right_trigger;
            buttonLT = this.gamepad1.left_trigger;
            a = this.gamepad1.a;
            b = this.gamepad1.b;
            x = this.gamepad1.x;
            if(runtime.seconds() >= 1 && buttonRB == true){
                velocityofcar = velocityofcar + 0.2;
                runtime.reset();
            }else if (runtime.seconds() >= 1 && buttonLB == true){
                velocityofcar = velocityofcar - 0.2;
                runtime.reset();
            }
            if(runtime.seconds() >= 1 && buttonRT >= 0.5){
                velocityofrotation = velocityofrotation + 0.1;
                runtime.reset();
            }else if (runtime.seconds() >= 1 && buttonLT >= 0.5){
                velocityofrotation = velocityofrotation - 0.1;
                runtime.reset();
            }
            /*pon = ((trtPowerx >= 0) ? 1 : -1);
            direction = Math.acos((trtPowery)/(Math.sqrt(trtPowerx*trtPowerx + trtPowery*trtPowery))) * (180/Math.PI);
            correctdir = (float) ((pon*direction > -182) ? pon*direction : correctdir);
            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX , AngleUnit.DEGREES);
            telemetry.addData("YAWZ", angles.firstAngle);
            telemetry.addData("YAWY", angles.secondAngle);
            telemetry.addData("YAWX", angles.thirdAngle);
            telemetry.addData("Direction", correctdir);
            telemetry.addData("velocity of car", velocityofcar);
            telemetry.addData("velocity of rotation", velocityofrotation);
            telemetry.addData("time in seconds", runtime.seconds());
            radianrobotdir  = angles.firstAngle*(Math.PI/180);
            absolutey = Math.cos(radianrobotdir)*tltPowery + Math.sin(radianrobotdir)*tltPowerx;
            absolutex = -Math.sin(radianrobotdir)*tltPowery + Math.cos(radianrobotdir)*tltPowerx;
            mecanum(absolutey, absolutex, trtPowery, -getRightDirection(correctdir, angles.firstAngle));
            telemetry.update();*/
            mecanum(tltPowery, -tltPowerx, trtPowery, trtPowerx);
            telemetry.addData("velocity of car", velocityofcar);
            telemetry.addData("velocity of rotation", velocityofrotation);
            telemetry.addData("time in seconds", runtime.seconds());
            altura = (a == true) ? altura - 0.001f : altura;
            altura = (x == true) ? altura + 0.001f : altura;
            telemetry.addData("brazos", altura);
            altura = (altura > 1) ? 1 : altura;
            altura = (altura < 0) ? 0 : altura;
            brazo_d.setPosition(altura);
            brazo_i.setPosition(altura);
            pinza.setPosition((b == true) ? 1 : 0.42f);
            telemetry.update();

        }
    }
    
    
    
    static void mecanum(double ly, double lx, double ry, double rx){
        frontLeftMotor.setPower(velocityofcar*(ly+(lx)+(-rx*velocityofrotation)));
        backLeftMotor.setPower(velocityofcar*(ly-(lx)+(-rx*velocityofrotation)));
        frontRightMotor.setPower(velocityofcar*(ly-(lx)-(-rx*velocityofrotation)));
        backRightMotor.setPower(velocityofcar*(ly+(lx)-(-rx*velocityofrotation)));
    }
    
    static double getRightDirection(double dir, double robotdir){
        if(Math.abs(dir) > 180 - 1.5 && Math.abs(dir) < 180 + 1.5){
            if(Math.abs(robotdir) < 180 - 1.5){
                return 1*(-Math.signum(robotdir));
            }else{
                return 0;
            }
        }else if(robotdir < dir - 1.5){
            return -1;
        } else if (robotdir > dir + 1.5){
            return 1;
        } else {
            return 0;
        }
    }
    
}