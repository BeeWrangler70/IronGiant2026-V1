package org.firstinspires.ftc.teamcode.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper class for managing an AprilTag detection pipeline using a webcam.
 * This class simplifies the initialization of the AprilTag processor and the Vision Portal,
 * and provides utility methods to access detection data.
 */
public class AprilTagWebcam {
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    @SuppressLint("DefaultLocale")
    public void displayDetectionTelemetry(AprilTagDetection tag) {
        if (tag == null) {
            telemetry.addLine(String.format("\nALIGN ROBOT CORRECTLY!!!"));
            return;
        }

        if (tag instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection singleDet = (AprilTagSingleDetection) tag;
            if (singleDet.metadata != null) {
                telemetry.addLine(String.format("\nROBOT CORRECTLY AIMED AT (ID %d) %s", singleDet.id, singleDet.metadata.name));
                telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                if (singleDet.ftcPose != null) {
                    telemetry.addLine(String.format("Distance to Target %6.1f (inch)", singleDet.ftcPose.range));
                    telemetry.addLine(String.format("Angle at Target %6.1f (inch)", singleDet.ftcPose.bearing));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                if (singleDet.center != null) {
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            }
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection tag : detectedTags) {
            if (tag instanceof AprilTagSingleDetection && ((AprilTagSingleDetection) tag).id == id) {
                return tag;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
