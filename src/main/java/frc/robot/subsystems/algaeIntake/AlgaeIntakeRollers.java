// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkLimitSwitch; 
import edu.wpi.first.wpilibj.DigitalSource; 


/** Class to run the rollers over CAN */
public class AlgaeIntakeRollersSubsystem extends SubsystemBase {
  private final SparkMax IntakeMotor;
  private boolean isRoller;
  private DigitalInput beambreak;
  SparkMaxConfig rollerConfig;
  boolean beambreakvalue;
  private SparkLimitSwitch LimitSwitchTop; 
  private SparkLimitSwitch LimitSwitchBottom; 
  

  public AlgaeIntakeRollersSubsystem() {
    // Set up the roller motor as a brushed motor
    if(isRoller==true){
    this.rollerConfig = new SparkMaxConfig();
    this.rollerConfig.voltageCompenspation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    this.rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    this.rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    else{
    this.IntakeMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);
    this.IntakeMotor.setCANTimeout(250);
    }

    this.IntakeMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);
    this.beambreak = new DigitalInput(3);
    this.beambreakvalue = false;
    this.LimitSwitchTop = new SparkLimitSwitch();
    this.LimitSwitchBottom = new SparkLimitSwitch();
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    this.IntakeMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    this.rollerConfig = new SparkMaxConfig();
    this.rollerConfig.voltageCompenspation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    this.rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    this.rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    this.beambreakvalue = getBeamBreakValue()
    this.LimitSwitchTopState=LimitSwitchTopisPressed();
    if (LimitSwitchTopState==true){

    }
    else {

    }
     }

  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse) {
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
  //Gets the beam break value - tells if there's an algae in the intake.
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
  