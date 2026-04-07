package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import swervelib.SwerveDrive;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    private final UsbCamera camera;
    private final CvSink cvSink;

    private final Mat frame = new Mat();
    private final Mat gray = new Mat();

    private final AprilTagDetector detector;
    private final AprilTagPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    // 🔥 CHANGE THIS LATER BASED ON YOUR ROBOT
    private final Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(0.25, 0.0, 0.4), // forward, left, up (meters)
            new Rotation3d(0, Math.toRadians(20), 0) // pitch
        );

    public VisionSubsystem(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        // Start camera
        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(15);

        cvSink = CameraServer.getVideo();

        // AprilTag detector
        detector = new AprilTagDetector();
        detector.addFamily("tag36h11");

        // 🔥 CAMERA CALIBRATION (TEMP VALUES — WILL FIX LATER)
        poseEstimator = new AprilTagPoseEstimator(
            new AprilTagPoseEstimator.Config(
                0.1651,   // tag size (meters)
                450, 450, // fx, fy
                160, 120  // cx, cy
            )
        );

        // Load field layout
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    @Override
    public void periodic() {

        if (cvSink.grabFrame(frame) == 0) {
            return;
        }

        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

        AprilTagDetection[] detections = detector.detect(gray);

        for (AprilTagDetection detection : detections) {

            int id = detection.getId();

            Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(id);
            if (tagPoseOpt.isEmpty()) continue;

            Transform3d cameraToTag = poseEstimator.estimate(detection);

            // Field -> Tag
            Pose3d fieldToTag = tagPoseOpt.get();

            // Field -> Camera
            Pose3d fieldToCamera =
                fieldToTag.transformBy(cameraToTag.inverse());

            // Field -> Robot
            Pose3d fieldToRobot =
                fieldToCamera.transformBy(robotToCamera.inverse());

            Pose2d robotPose = fieldToRobot.toPose2d();

            // 🚫 Basic filter (VERY IMPORTANT)
            if (!isValidPose(robotPose)) continue;

            double distance = cameraToTag.getTranslation().getNorm();

            // Closer = more trust
            Matrix<N3, N1> stdDevs =
                distance < 2
                    ? VecBuilder.fill(0.4, 0.4, 0.7)
                    : VecBuilder.fill(1.2, 1.2, 1.5);
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision X", robotPose.getX());
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision Y", robotPose.getY());
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision Heading", robotPose.getRotation().getDegrees());

            // 🔥 THIS IS THE MAGIC LINE
            swerveDrive.addVisionMeasurement(
                robotPose,
                Timer.getFPGATimestamp(),
                stdDevs
            );

            break; // only use first good tag
        }
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Tag Count", detections.length);
    }

    private boolean isValidPose(Pose2d pose) {
        return Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians());
    }
}