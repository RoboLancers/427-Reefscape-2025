// package frc.robot.subsystems;
// import java.util.List;
// import java.util.Optional;

// import javax.lang.model.util.Elements;
// import javax.naming.spi.DirStateFactory.Result;
// import javax.xml.stream.events.EndElement;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class VisionSubsystem extends SubsystemBase {
//     PhotonCamera camera;
//     PhotonPoseEstimator photonPoseEstimator;
//     List<PhotonPipelineResult> unreadResults;
//     EstimatedRobotPose estimatedRobotPose;
//     Pose3d referencePose;
    

//     public VisionSubsystem() {
//         camera = new PhotonCamera("photonvision");
//         photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.robotToCam);
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putBoolean("Front Camera is Connected", this.camera.isConnected()); 

//         if (!this.camera.isConnected()) return; 

//         unreadResults = camera.getAllUnreadResults();

//         unreadResults.get(unreadResults.size()-1);
//         // [1, 2, 5] elements (3)
//         //  0  1  2  index    (2)
//         //Last index = number of elements -1, last apriltag detected in last frame

//         Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update(unreadResults.get(unreadResults.size()-1));
//         //Most recent estimated pose
//         //Optional: if you do or dont have estimated already


//         if (estimate.isPresent()) {
//             estimatedRobotPose = estimate.get();
//             //If have estimated pose, get reference pose from it

//         } else {  
//            return;
//         }  //If no estimated pose, return to periodic

//         photonPoseEstimator.setReferencePose(estimatedRobotPose.estimatedPose);
//         //Gets pose from PhotonVision and use WPILib's Pose3d to set as a reference position
//         //Gets refrence pose from last pose

        

        
//     }

    
// }