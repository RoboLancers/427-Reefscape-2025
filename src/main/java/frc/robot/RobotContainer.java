// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
 import frc.robot.commands.AutoCommand;
// import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Intake.CANRollerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;



import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.WaitCommand;
//import frc.robot.subsystems.algaeIntake.AlgaeIntakeRollersSubsystem;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.WaitCommand;

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
  

  //private final AlgaeIntakeRollersSubsystem algaeRollerSubsystem = new AlgaeIntakeRollersSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
        OperatorConstants.OPERATOR_CONTROLLER_PORT);

      public DriveSubsystem driveSubsystem;

      public CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();
       public VisionSubsystem visionSubsystem = new VisionSubsystem();
      private SendableChooser<Command> autoChooser = new SendableChooser<>();

  
   //The container for the robot. Contains subsystems, OI devices, and commands.
   
  public RobotContainer() {
    // Set up command bindings


    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Coral roller and wait command for auto.
    NamedCommands.registerCommand("Coral_Score",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Coral_Score_Bottom",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Coral_Score_Bottom_2",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Coral_Score_Bottom_3",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Coral_Score_Top",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Coral_Score_Top_2",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Coral_Score_Top_3",new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));
    NamedCommands.registerCommand("Wait_Coral_Top", new WaitCommand(1.0)); 
    NamedCommands.registerCommand("Wait_Coral_Top_2", new WaitCommand(1.0)); 
    NamedCommands.registerCommand("Wait_Coral_Bottom", new WaitCommand(1.0)); 
    NamedCommands.registerCommand("Wait_Coral_Bottom_2", new WaitCommand(1.0)); 
    // Logging callback for current robot pose
      // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      //     // Do whatever you want with the pose here
      //     field.setRobotPose(pose);
      // });


      // Logging callback for target robot pose
      // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      //     // Do whatever you want with the pose here
      //     field.getObject("target pose").setPose(pose);
      // });


      // Logging callback for the active path, this is sent as a list of poses
      // PathPlannerLogging.setLogActivePathCallback((poses) -> {
      //     // Do whatever you want with the poses here
      //     field.getObject("path").setPoses(poses);
      // });
   


    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    //autoChooser.addOption

    try {
      driveSubsystem = new DriveSubsystem();
    } catch (IOException e) {

      e.printStackTrace();
    }

    // // Set up command bindings

    configureBindings();
    
 
    //rollers.setDefaultCommand(rollers.setMechanismVoltage(Volts.of(0)));
    //algaeRollerSubsystem.setDefaultCommand(rollers.setMechanismVoltage(Volts.of(0)))
    // Set up command bindings
    //configureBindings();
    
    //autoChooser.setDefaultOption("Autonomous", new AutoCommand(driveSubsystemCAN));

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

    driverController.b().whileTrue(driveSubsystem.tune(
      () -> driverController.getLeftX(), 
      () -> driverController.getLeftY(),
      () -> driverController.getRightX()
      )
      );
      driverController.x().onTrue(Commands.runOnce(()->driveSubsystem.resetPose(new Pose2d())));

     driverController.a()
      .whileTrue(new RollerCommand(() -> 0, () -> RollerConstants.ROLLER_EJECT_VALUE, rollerSubsystem));
      operatorController.a()
      .whileTrue(new RollerCommand(() -> 0, () -> RollerConstants.ROLLER_EJECT_VALUE, rollerSubsystem));
     //  .whileTrue(rollerSubsystem.runRollerCommand(RollerConstants.ROLLER_EJECT_VALUE, 0));
    if(RobotBase.isSimulation()){
      driveSubsystem.resetPose(new Pose2d(2,2,new Rotation2d()));
    } 
    // Set the A button to run the "RollerCommand" command with a fixed
    // value ejecting the gamepiece while the button is held

    // befo
    //operatorController.b()
    //    .whileTrue(new AlgaeCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, algaeRollerSubsystem));
    
    //operatorController.leftTrigger().whileTrue(new AlgaeCommand(()->0.44,()->0,algaeRollerSubsystem ));
    //    operatorController.rightTrigger().whileTrue(new AlgaeCommand(()->0,()->0.44,algaeRollerSubsystem ));
    // Set the default command for the drive subsystem to an instance of the
    // DriveCommand with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). Similarly for the X axis where we need to flip the value so the
    // joystick matches the WPILib convention of counter-clockwise positive

    // translationx and translation y to left joystick 
    // angular rotaiton to right x joystick
    driveSubsystem.setDefaultCommand(
      driveSubsystem.driveCommand( 
        () ->-MathUtil.applyDeadband(driverController.getLeftY(), 0.05), 
        () ->-MathUtil.applyDeadband(driverController.getLeftX(), 0.05),
        () ->-MathUtil.applyDeadband(driverController.getRightX(), 0.05)
        )
        );

        boolean isCompetition = true;

    rollerSubsystem.setDefaultCommand(
      new RollerCommand(
        () -> 0,
        () -> 0,
        rollerSubsystem)
      );


      autoChooser = AutoBuilder.buildAutoChooser("Center Auto");
 

      //autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    //   (stream) -> isCompetition
    //     ? stream.filter(auto -> auto.getName().startsWith("comp"))
    //     : stream
    // );

 


    SmartDashboard.putData("Auto Chooser", autoChooser);

  }
  
   // Use this to pass the autonomous command to the main {@link Robot} class.
   
   // @return the command to run in autonomous
   
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    try{
      // Load the path you want to follow using its name in the GUI
    //PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
      return autoChooser.getSelected();


      // Create a path following command using AutoBuilder. This will also trigger event markers.
      
    //return AutoBuilder.followPath(path);
  } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
}
}
  

