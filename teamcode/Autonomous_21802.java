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
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
import org.firstinspires.ftc.teamcode.AprilTagIdCode;
import java.util.ArrayList;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
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

public class Autonomous_21802 extends LinearOpMode {
    static private DcMotor backLeftMotor;
    static private DcMotor backRightMotor;
    static private Blinker control_Hub;
    static private Blinker expansion_Hub_2;
    static private DcMotor frontLeftMotor;
    static private DcMotor frontRightMotor;
    static private Servo brazo_d;
    static private Servo brazo_i;
    static private DcMotor corehex_d;
    static private DcMotor corehex_i;
    static private IMU imu;
    static private Orientation angles;
    static private IMU imu_expansion;
    static private Orientation angles_expansion;
    static private double velocityofcar = 0.8;
    static private ElapsedTime runtime = new ElapsedTime();
    static private ElapsedTime startTime = new ElapsedTime();
    static private ElapsedTime powerTime = new ElapsedTime();
    static private Servo pinza;
    static float time_x = 1.50f;
    static float time_y = 1.05f;
    static IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        AprilTagIdCode.BlocksContext detector;
        ArrayList AllDetections;
        int numberOfDetections;
        AprilTagDetection singleDetection;
        double detectedID = 3;
        
        detector = AprilTagIdCode.createAprilTagDetector(hardwareMap, "Webcam 1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        imu_expansion = hardwareMap.get(IMU.class, "imu_expansion");
        brazo_d = hardwareMap.get(Servo.class, "brazo_d");
        brazo_i = hardwareMap.get(Servo.class, "brazo_i");
        corehex_d = hardwareMap.get(DcMotor.class, "corehex_d");
        corehex_i = hardwareMap.get(DcMotor.class, "corehex_i");
        pinza = hardwareMap.get(Servo.class, "pinza");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navX");
        gyro = (IntegratingGyroscope)navxMicro;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.resetYaw();
        double radianrobotdir = 0;
        float robotlocation_x = 0;
        float robotlocation_y = 0;
        float buttonRT = 0;
        float buttonLT = 0;
        boolean buttonRB = false;
        boolean buttonLB = false;
        boolean button_a = false;
        boolean terminar = false;
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AprilTagIdCode.startAprilTagDetector(detector, 640, 480);
        
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        startTime.reset();
        powerTime.reset();
        imu_expansion.resetYaw();
        
        
        while (opModeIsActive() && detectedID == 3) {
            AllDetections = AprilTagIdCode.getAllDetections(detector);
            numberOfDetections = AprilTagIdCode.getHowManyDetections(AllDetections);
            if (numberOfDetections == 0) {
                telemetry.addData("No detections", "KEEP LOOKING");
            } else if (numberOfDetections >= 2) {
                telemetry.addData("Multiple detections", "WAIT FOR ONE ONLY");
            } else {
                singleDetection = AprilTagIdCode.getOneDetection(AllDetections, 0);
                detectedID = AprilTagIdCode.getID(singleDetection);
                telemetry.addData("key", detectedID);
            }
            telemetry.update();
        }
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            angles_expansion = imu_expansion.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX , AngleUnit.DEGREES);
            telemetry.addData("YAWZ", angles_expansion.firstAngle);
            // 560 ticks for revolution.
            telemetry.addData("Front Left Motor", frontLeftMotor.getCurrentPosition()/560f);
            telemetry.addData("Front Right Motor", frontRightMotor.getCurrentPosition()/560f);
            telemetry.addData("Back Left Motor", backLeftMotor.getCurrentPosition()/560f);
            telemetry.addData("Back Right Motor", backRightMotor.getCurrentPosition()/560f);
            telemetry.addData("runTime", runtime.seconds());
            telemetry.addData("startTime", startTime.seconds());
            telemetry.addData("powerTime", powerTime.seconds());
            telemetry.addData("time_x", time_x);
            telemetry.addData("time_y", time_y);
            telemetry.addData("key", detectedID);
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

            telemetry.addLine()
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
            if(terminar == false){
                goTo(0,0.1f,0,0);
            }
            /*goTo(1,1,0,0);
            goTo(-1,0,0,0);*/
            
            /*buttonRT = this.gamepad1.right_trigger;
            buttonLT = this.gamepad1.left_trigger;
            buttonRB = this.gamepad1.right_bumper;
            buttonLB = this.gamepad1.left_bumper;
            button_a = this.gamepad1.a;
            if(buttonRT > 0.5 && powerTime.seconds() >= 0.5){
                time_x = time_x + 0.01f;
                powerTime.reset();
                
            }else if(buttonLT > 0.5 && powerTime.seconds() >= 0.5){
                time_x = time_x - 0.01f;
                powerTime.reset();
            }
            if(buttonRB == true && powerTime.seconds() >= 0.5){
                time_y = time_y + 0.01f;
                powerTime.reset();
            }else if(buttonLB == true && powerTime.seconds() >= 0.5){
                time_y = time_y - 0.01f;
                powerTime.reset();
            }
            if(button_a == true){
                telemetry.addData("Start", "hola");
                goTo(3,2,0,0);
                telemetry.update();
            }*/
            
            if(detectedID == 0 && terminar == false){
                runtime.reset();
                while(runtime.seconds() <= time_x){
                    mecanum(0, 0.5, 0, getRightDirection(0, 0));
                    info();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                runtime.reset();
                while(runtime.seconds() <= time_y){
                    mecanum(-0.5, 0, 0, getRightDirection(0, 0));
                    info();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                runtime.reset();
                terminar = true;
            }else if(detectedID == 1 && terminar == false){
                runtime.reset();
                /*while(runtime.seconds() <= time_x){
                    mecanum(0, 0.5, 0, getRightDirection(0, angles.firstAngle));
                    info();
                }*/
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                runtime.reset();
                while(runtime.seconds() <= time_y){
                    mecanum(-0.5, 0, 0, getRightDirection(0, 0));
                    info();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                runtime.reset();
                terminar = true;
            }else if(detectedID == 2 && terminar == false){
                runtime.reset();
                while(runtime.seconds() <= time_x){
                    mecanum(0, -0.5, 0, getRightDirection(0, 0));
                    info();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                runtime.reset();
                while(runtime.seconds() <= time_y){
                    mecanum(-0.5, 0, 0, getRightDirection(0, 0));
                    info();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                runtime.reset();
                terminar = true;
            }
            telemetry.addData("Stop", "adios");
            telemetry.update();
            idle();
        }
    }
    
    static void mecanum(double ly, double lx, double ry, double rx){
        frontLeftMotor.setPower(velocityofcar*(ly+lx-rx));
        backLeftMotor.setPower(velocityofcar*(ly-lx-rx));
        frontRightMotor.setPower(velocityofcar*(ly-lx+rx));
        backRightMotor.setPower(velocityofcar*(ly+lx+rx));
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
    // 23.5 inches, 141 inches = completo de lado a lado.
    static void goTo(float x, float y, float x_robot, float y_robot){
        float direction_x = x-x_robot;
        float direction_y = y-y_robot;
        int pon = (direction_x >= 0) ? 1 : -1;
        int pin = (direction_y >= 0) ? 1 : -1;
        runtime.reset();
        while(runtime.seconds() <= Math.abs(direction_x)*time_x){
            mecanum(0, pon*-0.5, 0, getRightDirection(0, angles_expansion.firstAngle));
            info();
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        runtime.reset();
        while(runtime.seconds() <= Math.abs(direction_y)*time_y){
            mecanum(pin*-0.5, 0, 0, getRightDirection(0, angles_expansion.firstAngle));
            info();
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        runtime.reset();
    }
    
    static private void info(){
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    
    static void alturaElevador(float altura){
        if(corehex_i.getCurrentPosition()/288f < altura-0.1){
            corehex_i.setPower(1);
            corehex_d.setPower(-1);
        }else if(corehex_i.getCurrentPosition()/288f > altura+0.1){
            corehex_i.setPower(-1);
            corehex_d.setPower(1);
        }else{
            corehex_i.setPower(0);
            corehex_d.setPower(0);
        }
    }
    
    static void brazo_function (double valor3){
        brazo_d.setPosition(valor3);
        brazo_i.setPosition(valor3);
    }
    
    static String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
