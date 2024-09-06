package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagCamera {
    private OpenCvCamera camera; //create objects for camera and apriltag detection
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final HardwareMap hardwareMap; 
    private final Telemetry telemetry;
    private final LinearOpMode linearOpMode;
    private final Gamepad gamepad;
    static final double INCH_PER_METER = 39.3700787; //exchange rate between inch and meter
    double fx = 578.272; //camera callibration
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.160; //declare apriltag size

    int left = 0;
    int right = 20; //used for the apriltag detector

    AprilTagDetection tagOfInterest = null; //create a null status of apriltag detection 

    public AprilTagCamera(LinearOpMode linearOpMode, Gamepad gamepad) { //create access for LinearOpModes 
        this.hardwareMap = linearOpMode.hardwareMap;
        this.telemetry = linearOpMode.telemetry;
        this.linearOpMode = linearOpMode;
        this.gamepad = gamepad;
    }

    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //get the camera resoure id
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId); //connect with the webcam
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy); //create an apriltag detection with tagsize and camera callibration

        camera.setPipeline(aprilTagDetectionPipeline); //specify the image processing pipeline that you want to use
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() { //allows other parts of the program to be executed while operating independently
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT); //enable camera streaming
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera error: " + errorCode); //print the error code on the driver hub if camera encounter an error
                telemetry.update();
            }
        });
        telemetry.setMsTransmissionInterval(50); //set minimum interval between telemetry transmissions
    } 

    public void trackAprilTag() {
        while (!linearOpMode.isStarted() && !linearOpMode.isStopRequested()) { 
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            boolean tagFound = false;
            if (!currentDetections.isEmpty()) { 
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id >= left && tag.id <= right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        gamepad.rumble(300); //rumble the gamepad in 300ms when the camera detected an apriltag
                        break;
                    }
                }
            }
            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:"); //print this line to driver hub when camera found a tag
                tagToTelemetry(tagOfInterest);
            }
            telemetry.update();
        }
        telemetry.update(); //update telemetry
    }
 
    private void tagToTelemetry(AprilTagDetection detection) { //print the x, y, yaw value to the driver hub
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * INCH_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * INCH_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
    }
}
