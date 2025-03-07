// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.AlgaeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode; 

/** Class to run the rollers over CAN */
public class AlgaeIntakeRollersSubsystem extends SubsystemBase {
  private final SparkMax pivotMotor;
  //remove isRoller
  private boolean isRoller;
  private DigitalInput beambreak;
  SparkMaxConfig rollerConfig;
  boolean beambreakvalue;
  private SparkLimitSwitch LimitSwitchTop; 
  private SparkLimitSwitch LimitSwitchBottom;
  private final SparkMax rollerMotor;
  private PIDController pidController = new PIDController(kP, kI, kD);
  private SparkAbsoluteEncoder absEncoder; 

  public AlgaeIntakeRollersSubsystem() {
    // Set up the roller motor as a brushed motor
    this.pivotMotor = new SparkMax(AlgaeConstants.PIVOT_MOTOR_ID, MotorType.kBrushed);
    this.pivotMotor.setCANTimeout(250);
    this.rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);
    this.rollerMotor.setCANTimeout(250);

    if(isRoller==true){
      // Create and apply configuration for roller motor. Voltage compensation helps
      // the roller behave the same as the battery
      // voltage dips. The current limit helps prevent breaker trips or burning out
      // the motor in the event the roller stalls.
      this.rollerConfig = new SparkMaxConfig();
      this.rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
      this.rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
      this.rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      this.rollerConfig = new SparkMaxConfig();
      this.rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
      this.rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
      this.pivotMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    this.beambreak = new DigitalInput(3);
    this.beambreakvalue = false;
    //this.LimitSwitchTop = new SparkLimitSwitch();
    //this.LimitSwitchBottom = new SparkLimitSwitch();
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
  }

  @Override
  public void periodic(){
    // The value of the beambreak
    this.beambreakvalue = getBeamBreakValue();
    this.beambreakvalue = getBeamBreakValue();
    //The variable of weather the top limit switch was pressed or not.
    //this.LimitSwitchTopState=LimitSwitchTopisPressed();
    // If the top limit switch is pressed then it will stop the intakemotor.

    //if (LimitSwitchTopState==true){
    //IntakeMotor.set(0,0);
    //}
     }

    // If the bottom limit switch is pressed then it will stop the intake motor.
     //if (LimitSwitchBottomisPressed()==true){
    //  IntakeMotor.set(0,0);
    // }
     // If the beambreak gets tripped then it will stop the roller motor.
     if (beambreakvalue == true){
      rollerMotor.set(0);
     }
  }
  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse){
    IntakeMotor.set(forward - reverse);
  }
  // Checks if the LimitSwitch top is pressed

  public boolean LimitSwitchTopisPressed(){
    return this.LimitSwitchTop.isPressed();
  }
  // Checks if the LimitSwitch bottom is pressed
  public boolean LimitSwitchBottomisPressed(){
    return this.LimitSwitchBottom.isPressed();
  }
  // Gets the beam break value - tells if there's an algae in the intake.
  public boolean getBeamBreakValue(){
    return this.beambreak.get();
  }
  // Checks which channel the diital input is coming from.
  public int getbeambreakChannel(){
    return this.beambreak.getChannel();
  }
  // Gets the trigger type of the analog.
  public int getAnalogTriggerTypeForRoutingbeambreak(){
    return this.beambreak.getAnalogTriggerTypeForRouting();
  }
  // Tells whether this is or isn't an analog trigger.
  public boolean beambreakisAnalogTrigger(){
    return this.beambreak.isAnalogTrigger();
  }
  // Gets the HAL handle for a specific source.
  public int beambreakgetPortHandleForRouting(){
    return this.beambreak.getPortHandleForRouting();
  }


  ArmFeedforward feedForward = new ArmFeedforward(0, 0, kG);

public Command intake(){
  return run(() -> {
    rollerMotor.setVoltage(kIntakeVoltage)
  });
}

public Command outtake(){
  return run(()-> {
    rollerMotor.setVoltage(kOuttakeVoltage)
  });
}

  public Command goToAngle(double targetAngle) {
    return run(() ->  {
       double measurement = absEncoder.getPosition() //gets current angle
       //targetAngle = the target angle you wanna go to
       pivotMotor.setVoltage(pidController.calculate(measurement, targetAngle) + feedForward.calculate(targetAngle, 0));
    });
  }

public Command tune() {
  Smartdashboard.putNumber("/AlgaeIntakeRollers/kP", 0);
  Smartdashboard.putNumber("/AlgaeIntakeRollers/kI", 0);
  Smartdashboard.putNumber("/AlgaeIntakeRollers/kD", 0);
  Smartdashboard.putNumber("/AlgaeIntakeRollers/kG", 0);
  Smartdashboard.putNumber("AlgaeIntakeRollers/desiredAngle", 0);

  return run(() -> { 
    this.pidController.set(Smartdashboard.putNumber("/AlgaeIntakeRollers/kP", 0), Smartdashboard.putNumber("/AlgaeIntakeRollers/kI", 0), Smartdashboard.putNumber("/AlgaeIntakeRollers/kG", 0));
    this.feedForward = new ArmFeedforward(0, Smartdashboard.putNumber("/AlgaeIntakeRollers/kG", 0), 0);
    goToAngle(Smartdashboard.putNumber("AlgaeIntakeRollers/desiredAngle", 0));
  });
}
