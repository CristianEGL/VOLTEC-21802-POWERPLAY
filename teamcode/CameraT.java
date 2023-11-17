package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.AprilTagIdCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "CameraT (Blocks to Java)")
public class CameraT extends LinearOpMode {

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    AprilTagIdCode.BlocksContext detector;
    ArrayList AllDetections;
    int numberOfDetections;
    AprilTagDetection singleDetection;
    double detectedID;

    // Put initialization blocks here.
    // Create a pipeline/webcam object for AprilTag ID code detection. Edit configured webcam name as needed.  Use this myBlock in INIT section of OpMode, before startAprilTagDetector.
    detector = AprilTagIdCode.createAprilTagDetector(hardwareMap, "Webcam 1");
    // Begin operating camera/stream/pipeline for AprilTag
    // detection. Must specify a resolution supported by the camera;
    // edit these default values as desired. Use this myBlock
    // in INIT section of OpMode, after createAprilTagDetector.
    AprilTagIdCode.startAprilTagDetector(detector, 640, 480);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Provide the data from any and all detetected AprilTags. Use this myBlock anywhere in the OpMode, after startAprilTagDetector.
        AllDetections = AprilTagIdCode.getAllDetections(detector);
        // Provide the number of detections in the current batch. Use this myBlock anywhere in the OpMode, after getAllDetections.
        numberOfDetections = AprilTagIdCode.getHowManyDetections(AllDetections);
        if (numberOfDetections == 0) {
          telemetry.addData("No detections", "KEEP LOOKING");
        } else if (numberOfDetections >= 2) {
          telemetry.addData("Multiple detections", "WAIT FOR ONE ONLY");
        } else {
          // Provide the data from only the first detection in the current batch.  This myBlock is helpful when you know there's only one AprilTag detected.  Use it anywhere, after getAllDetections. This will crash if the input detections list is empty.
          singleDetection = AprilTagIdCode.getOneDetection(AllDetections, 0);
          // Provide the AprilTag ID code from the designated detection. Use this myBlock anywhere, after getOneDetection.
          detectedID = AprilTagIdCode.getID(singleDetection);
          telemetry.addData("key", detectedID);
        }
        telemetry.update();
      }
    }
  }
}
