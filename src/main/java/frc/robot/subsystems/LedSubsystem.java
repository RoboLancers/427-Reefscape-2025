package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 0;
    private static final int kLength = 0;


    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(63);

    public LEDSubsystem() {
        LEDPattern red = LEDPattern.solid(Color.kRed); 
    }

@Override 
public void periodic() {
        //Periodically sends latest LED data to LED strip
    m_led.setData(m_buffer);
}

public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
    }
}