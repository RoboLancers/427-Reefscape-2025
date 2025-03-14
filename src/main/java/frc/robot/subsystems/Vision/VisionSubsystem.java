package frc.robot.subsystems.Vision;
import java.nio.channels.UnresolvedAddressException;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

 
//Determines position of robot based on AprilTags
public class VisionSubsystem extends SubsystemBase {
    
    public static VisionSubsystem instance;
    
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    PhotonPipelineResult latestResults;
    EstimatedRobotPose estimatedPose;
    Pose3d referencePose;

    private Field2d visionField = new Field2d(); 
    
    public VisionSubsystem() {
        PhotonCamera.setVersionCheckEnabled(true);
        
        camera = new PhotonCamera("USB_Camera");
        
        photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.robotToCam);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Front Camera is Connected", this.camera.isConnected());

        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        
        if (unreadResults.size() == 0) return; 
            // [1, 2, 5] elements (3)
            //  0  1  2  index    (2)
        //Last index = number of elements -1, LAST APRILTAG detected in last frame
        latestResults = unreadResults.get(unreadResults.size()-1);

        Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update(unreadResults.get(unreadResults.size()-1));
        SmartDashboard.putBoolean("Estimator Present", estimate.isPresent());    
        //Most recent estimated pose
        //Optional: if you do or dont have estimated already
        if (estimate.isPresent()) {
            estimatedPose = estimate.get();
            //If have estimated pose, get reference pose from it
        } else {  
         
            return;
        } //If no estimated pose, return to periodic

        photonPoseEstimator.setReferencePose(estimatedPose.estimatedPose);
        referencePose = estimatedPose.estimatedPose;
        //Gets pose from PhotonVision and use WPILib's Pose3d to set as a reference position (LAST POSITION)
        //Gets refrence pose from last pose

        //Logs X,Y, and Z values onto the SmartDashboard
        Constants.VisionConstants.aprilTagFieldLayout.getTagPose(getBestAprilTagID()).get().getX();
        Constants.VisionConstants.aprilTagFieldLayout.getTagPose(getBestAprilTagID()).get().getY();
        Constants.VisionConstants.aprilTagFieldLayout.getTagPose(getBestAprilTagID()).get().getZ();

        if (getCurrentPose3d() != null) {
            SmartDashboard.putNumber("VisionPoseX", getReferencePose().getX());
            SmartDashboard.putNumber("VisionPoseY", getReferencePose().getY());
            SmartDashboard.putNumber("VisionPoseZ", getReferencePose().getZ());
        }

        if (getAprilTagPos(getBestAprilTagID()) != null) {
            SmartDashboard.putNumber("VisionTargetX", getAprilTagPos(getBestAprilTagID()).getX());
            SmartDashboard.putNumber("VisionTargetY", getAprilTagPos(getBestAprilTagID()).getY());
            SmartDashboard.putNumber("VisionTargetZ", getAprilTagPos(getBestAprilTagID()).getZ());    
        }

        visionField.setRobotPose(getReferencePose().toPose2d());
        SmartDashboard.putData("Vision Field", visionField);
    }

    public static VisionSubsystem getInstance() {
        return instance;
    }

    private Pose3d getReferencePose() {
        return referencePose;
    }

    private Pose3d getAprilTagPos(int aprilTagID) {
        return Constants.VisionConstants.aprilTagFieldLayout.getTagPose(aprilTagID).get();
    }

    private int getBestAprilTagID() {
        return latestResults.getBestTarget().getFiducialId();
    }

    private EstimatedRobotPose getCurrentPose3d() {
        return estimatedPose;
    }

    private double getDistanceToAprilTag() {
        Pose3d aprilTagPose = getAprilTagPos(getBestAprilTagID());
        if (aprilTagPose == null) return 1000;
        return getDistanceBetweenPose3d(aprilTagPose, getCurrentPose3d().estimatedPose);
        //need getCurrentPose3d().estimatedPose because currentpose3d accesses the varibles estimatedpose contains, as estimated itself doesn't show them
    }

    private double getDistanceBetweenPose3d(Pose3d firstPose, Pose3d secondPose) {
        double xDifferenceSquared = Math.pow(secondPose.getX() - firstPose.getX(), 2);
        double yDifferenceSquared = Math.pow(secondPose.getY() - firstPose.getY(), 2);
        double zDifferenceSquared = Math.pow(secondPose.getZ() - firstPose.getZ(), 2);
        return Math.sqrt(xDifferenceSquared + yDifferenceSquared + zDifferenceSquared);
    }

    private double getDistanceBetweenPose2d(Pose3d firstPose, Pose3d secondPose) {
        double xDifferenceSquared = Math.pow(secondPose.getX() - firstPose.getX(), 2);
        double yDifferenceSquared = Math.pow(secondPose.getY() - firstPose.getY(), 2);
        return Math.sqrt(xDifferenceSquared + yDifferenceSquared);
    }

    private Rotation3d getRotation3d() {
        return getCurrentPose3d().estimatedPose.getRotation();
    }
}
