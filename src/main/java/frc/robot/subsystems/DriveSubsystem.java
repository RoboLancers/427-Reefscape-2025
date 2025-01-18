package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
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
