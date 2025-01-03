package frc.robot.utils;
import edu.wpi.first.math.geometry.Pose2d;


/** An object that contains all relevant information for  sending apriltag info */
    public class ApriltagMeasurement {
                
        // Variable definitions
        public Pose2d pose;
        public long timeStamp; 
        public double distance;
        public int tagID;
        
        /**
         * An object that contains all relevant information for  sending apriltag info
         * @param pose
         * @param timeStamp
         * @param distance
         * @param tagID
         */
        public ApriltagMeasurement (
            Pose2d pose, 
            long timeStamp, 
            double distance,
            int tagID) {
                    this.pose = pose;
                    this.timeStamp = timeStamp;
                    this.distance = distance;
                    this.tagID = tagID;
                }
}