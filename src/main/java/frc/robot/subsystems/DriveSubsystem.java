package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

=======
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.wpilibj.DriverStation;
>>>>>>> bba00aca40a5e31611eb832f8913aba406971f40
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
<<<<<<< HEAD
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveSubsystem extends SubsystemBase{
    SwerveDrive swerveDrive;

    public DriveSubsystem() throws IOException{
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxspeed);

        swerveDrive.swerveController.setMaximumChassisAngularVelocity(DriveConstants.maxAngle);
    }
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * DriveConstants.maxspeed,
                                            translationY.getAsDouble() * DriveConstants.maxspeed),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      });
    }
}
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;


public class DriveSubsystem extends SubsystemBase{
  SwerveDrive swerveDrive;
  RobotConfig config;
 


  public DriveSubsystem() throws IOException{
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxSpeed);


    swerveDrive.swerveController.setMaximumChassisAngularVelocity(DriveConstants.maxAngularSpeed);


    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }


    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        () -> {


          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
            this
        );
      }
         
  public Command followPathCommand(String pathName) {
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);


      return AutoBuilder.followPath(path);


    // return new FollowPathCommand(
    //   path,
    //   this::getPose, // Robot pose supplier
    //   this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //   (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
    //   new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //           new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //           new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //   ),
    //   config,
    //   () -> {
    //     // Boolean supplier that controls when the path will be mirrored for the red alliance
    //     // This will flip the path being followed to the red side of the field.
    //     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE


    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
    //   this // Reference to this subsystem to set requirements
    //   );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }


  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * DriveConstants.maxSpeed,
                                          translationY.getAsDouble() * DriveConstants.maxSpeed),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }


  public Pose2d getPose() {
    return swerveDrive.getPose();
  }


  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }


  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }


  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }
}

>>>>>>> bba00aca40a5e31611eb832f8913aba406971f40
