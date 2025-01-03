package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;


public class Vision extends SubsystemBase {
    
    private static PhotonCamera camera;
	
    public Vision() {
        camera = new PhotonCamera(Constants.Vision.cameraID);
    }

    

    public Rotation2d ringDirection(){
		// Gets target object orientation from orange photonvision
            if(checkForTarget()){
                return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getYaw());
            } else {
                return Rotation2d.fromDegrees(0);
            }
    }

	public boolean checkForTarget(){
		var result = camera.getLatestResult();//get a frame from the camera
		boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
		return hasTargets;
	}

    public double getTargetID() {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        int targetID = target.getFiducialId();
        return targetID;
    }

}