package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.w3c.dom.css.RGBColor;

import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LedLights extends SubsystemBase {
    private static final int kPort = RobotMap.LEDS_ID;
    private static final int kLength = RobotMap.LEDS_LENGTH * RobotMap.LEDS_PER_METER;
    private static final Distance kLedSpacing = Meters.of((double) RobotMap.LEDS_LENGTH / RobotMap.LEDS_PER_METER);

    private static final LEDPattern RAINBOW_PATTERN = LEDPattern.rainbow(200, 100)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(12), kLedSpacing);

    private static final LEDPattern FLASH_PATTERN = LEDPattern.steps(Map.of(0,returnRGBToGRB(0,255,255),0.25,Color.kGreen,0.5,returnRGBToGRB(255,255,0),0.75,Color.kGreen))
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(20), kLedSpacing);

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private LEDPattern currentPattern;

    public LedLights() {
        leds = new AddressableLED(kPort);
        ledBuffer = new AddressableLEDBuffer(kLength);
        leds.setLength(kLength);
        leds.start();

        setPattern(FLASH_PATTERN);
    }

    @Override
    public void periodic() {
        if (currentPattern != null) {
            currentPattern.applyTo(ledBuffer);
            leds.setData(ledBuffer);
        }

        assert currentPattern != null;
        SmartDashboard.putString("Current Pattern",currentPattern.toString());
    }

    public void setPattern(LEDPattern pattern) {
        if (pattern != currentPattern) {  // Only update if the pattern has changed
            this.currentPattern = pattern;
        }
    }

    public LEDPattern getCurrentPattern() {
        return currentPattern;
    }

    public LEDPattern getRainbowPattern() {
        return RAINBOW_PATTERN;
    }

    public LEDPattern getFlashPattern(){
        return FLASH_PATTERN;
    }

    public static Color returnRGBToGRB(double red, double green, double blue){
        return new Color(green,red,blue);
    }
}
