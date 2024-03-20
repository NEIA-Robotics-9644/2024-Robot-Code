package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{
    public VisionSubsystem(){
        camera = new PhotonCamera("VisionCam");
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
}
