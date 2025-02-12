/* TO-DO LIST:
Decoratory state.
Algae-inside-the-robot state.
Climb-is-ready state.
*/

/* ^|'-| Good Resources For Understanding: |-'|^
https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html#the-java-units-library
*/


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedSubsystem extends SubsystemBase {

  private static final int kPort_PWM = 9;
  private static final int kLength_LEDSTRIP = 63;

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  // :: List of patterns for the robot:
  LEDPattern stateOffPattern = LEDPattern.solid(Color.kBlack).atBrightness(Percent.of(25));
  LEDPattern stateDecoratoryPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kAqua, Color.kAquamarine);
  LEDPattern stateAlgaeInsidePattern = LEDPattern.solid(Color.kBlue);
  LEDPattern stateCclimbReadyPattern = LEDPattern.solid(Color.kCoral);

  public LedSubsystem() {
    ledStrip = new AddressableLED(kPort_PWM);
    ledBuffer = new AddressableLEDBuffer(kLength_LEDSTRIP);
    ledStrip.setLength(kLength_LEDSTRIP);
    ledStrip.start();
    
    // Create the buffer
    AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(0);

    @SuppressWarnings("unused")
    AddressableLEDBufferView m_left = m_buffer.createView(0, 0);
    @SuppressWarnings("unused")
	  AddressableLEDBufferView m_right = m_buffer.createView(0, 0).reversed();

    setDefaultCommand(runPattern(stateOffPattern).withName("Off"));
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(ledBuffer));
  }

}

