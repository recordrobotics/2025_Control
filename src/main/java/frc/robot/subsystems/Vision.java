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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;


public class Vision extends SubsystemBase {
    
    private static PhotonCamera left;
    private static PhotonCamera right;
	
    public Vision() {
        left = new PhotonCamera(Constants.Vision.leftCameraID);
        right = new PhotonCamera(Constants.Vision.rightCameraID);
    }

    // public void periodic()  {
    //     // Gets a frame from the camera
	// 	var result = camera.getLatestResult();
    // }


    public Translation2d ringLocation(){
            if(checkForTarget()){
                double leftSlope = Math.tan(left.getLatestResult().getBestTarget().getYaw() * Math.PI / 180);
                double rightSlope = Math.tan(right.getLatestResult().getBestTarget().getYaw() * Math.PI / 180);
                double xPos = (Constants.Vision.cameraWidth * (leftSlope + rightSlope))/(leftSlope - rightSlope);
                double yPos = leftSlope * (xPos - (Constants.Vision.cameraWidth / 2));
                return new Translation2d(xPos, yPos + Constants.Vision.robotToCam.getY());
            } else {
                return new Translation2d(0, 0);
            }
    }

	public boolean checkForTarget(){
		return left.getLatestResult().hasTargets() && right.getLatestResult().hasTargets();
	}
}