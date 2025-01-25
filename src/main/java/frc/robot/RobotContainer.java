// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Field2d field;
  
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

      public DriveSubsystem driveSubsystem;
      public CANDriveSubsystem driveSubsystemCAN;

       // The autonomous chooser
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up command bindings

    field = new Field2d();
    SmartDashboard.putData("Field", field);


    // Logging callback for current robot pose
      PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
      });


      // Logging callback for target robot pose
      PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
      });


      // Logging callback for the active path, this is sent as a list of poses
      PathPlannerLogging.setLogActivePathCallback((poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
      });
   
    // Set up command bindings
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption

    try {
      driveSubsystem = new DriveSubsystem();
    } catch (IOException e) {

      e.printStackTrace();
    }
    
    autoChooser.setDefaultOption("Autonomous", new AutoCommand(driveSubsystemCAN));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driverController.a().whileTrue(driveSubsystem.tune(
      () -> driverController.getLeftX(), 
      () -> driverController.getLeftY(),
      () -> driverController.getRightX()
      )
      );
    // Set the A button to run the "RollerCommand" command with a fixed
    // value ejecting the gamepiece while the button is held


    // Set the default command for the drive subsystem to an instance of the
    // DriveCommand with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). Similarly for the X axis where we need to flip the value so the
    // joystick matches the WPILib convention of counter-clockwise positive

    //translationx and translatioin y to left joystick 
    // angular rotaiton to right x joystick
    driveSubsystem.setDefaultCommand(
      driveSubsystem.driveCommand( 
        () -> driverController.getLeftX(), 
        () -> driverController.getLeftY(),
        () -> driverController.getRightX()
        )
        );

        boolean isCompetition = true;


        autoChooser = AutoBuilder.buildAutoChooser("CoralThenStation");
 
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
 


    SmartDashboard.putData("Auto Chooser", autoChooser);

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    try{
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");


      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
  } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
