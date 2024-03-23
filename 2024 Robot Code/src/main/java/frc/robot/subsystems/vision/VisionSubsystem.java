/*

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{

    PhotonCamera camera;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(Units.degreesToRadians(180), 0, 0));
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

    //public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //    return photonPoseEstimator.update();
    //}

    public VisionSubsystem(){
        //camera = new PhotonCamera("VisionCam");
    }

    @Override
    public void periodic() {
        // Get the camera
        
        // Get the latest result
        var result = camera.getLatestResult();
        // Check for targets
        if (result.hasTargets()) {
            System.out.println("Found a target");
            // Get the best target
            var target = result.getBestTarget();
            // Get target info
            var yaw = target.getYaw();
            var pitch = target.getPitch();
            var camToTarget = target.getBestCameraToTarget();
            SmartDashboard.putNumber("Target yaw", yaw);
            SmartDashboard.putNumber("Target pitch", pitch);
            SmartDashboard.putString("Target transform", camToTarget.toString());
      
            SmartDashboard.putBoolean("Targeting april tag", true);
        } 
        else {
            SmartDashboard.putBoolean("Targeting april tag", false);
        }
    }
}

*/
