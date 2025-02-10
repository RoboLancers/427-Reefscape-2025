package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.PIDFConfig;
import swervelib.SwerveModule;


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

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
/* 
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
*/
  }
               
  public Command followPathCommand(String pathName) {
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
      
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
        // Make the robot move
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * DriveConstants.maxSpeed,
                                            translationY.getAsDouble() * DriveConstants.maxSpeed),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      });
    }

    public Command tune(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
        SmartDashboard.putNumber("SwerveModuleVelocitykP", 0);
        SmartDashboard.putNumber("SwerveModuleVelocitykI", 0);
        SmartDashboard.putNumber("SwerveModuleVelocitykD", 0);

        SmartDashboard.putNumber("SwerveModuleAnglekP", 0);
        SmartDashboard.putNumber("SwerveModuleAnglekI", 0);
        SmartDashboard.putNumber("SwerveModuleAnglekD", 0);

        return runOnce(() -> {
            SwerveModule[] modules = swerveDrive.swerveDriveConfiguration.modules;

            for (int i = 0; i < modules.length; i++) {
                PIDFConfig velocity = modules[i].configuration.velocityPIDF;
                PIDFConfig angle = modules[i].configuration.anglePIDF;


                SmartDashboard.putNumber("SwerveModuleDriveMotorVelocityMetersPerSecond", 0);
                SmartDashboard.putNumber("SwerveModuleAngleMotorPosition", 0);
                

                velocity.p = SmartDashboard.getNumber("SwerveModuleVelocitykP", 0);
                velocity.i = SmartDashboard.getNumber("SwerveModuleVelocitykI", 0);
                velocity.d = SmartDashboard.getNumber("SwerveModuleVelocitykD", 0);
    
                angle.p = SmartDashboard.getNumber("SwerveModuleAnglekP", 0);
                angle.i = SmartDashboard.getNumber("SwerveModuleAnglekI", 0);
                angle.d = SmartDashboard.getNumber("SwerveModuleAnglekD", 0);
            }
        }).andThen(
            driveCommand(translationX, translationY, angularRotationX)
        );
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
