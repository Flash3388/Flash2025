package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import org.w3c.dom.css.RGBColor;

import java.util.Map;

import static edu.wpi.first.units.Units.*;

public class LedLights extends SubsystemBase {
    private static final int kPort = RobotMap.LEDS_ID;
    private static final int kLength = RobotMap.LEDS_LENGTH * RobotMap.LEDS_PER_METER;
    private static final Distance kLedSpacing = Meters.of((double) RobotMap.LEDS_LENGTH / RobotMap.LEDS_PER_METER);

    private static final LEDPattern RAINBOW_PATTERN = LEDPattern.rainbow(255, 255);

    private static final LEDPattern FLASH_PATTERN = LEDPattern.steps(Map.of(0,returnRGBToGRB(255,200,0),0.25,returnRGBToGRB(200,0,0),0.5,returnRGBToGRB(255,200,0),0.75,returnRGBToGRB(200,0,0)));
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
            currentPattern.atBrightness(Percent.of(10)).applyTo(ledBuffer);
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
        return DriverStation.isDisabled() ? RAINBOW_PATTERN : RAINBOW_PATTERN.scrollAtAbsoluteSpeed(MetersPerSecond.of(15),kLedSpacing);
    }

    public LEDPattern getFlashPattern(){
        return DriverStation.isDisabled() ? FLASH_PATTERN : FLASH_PATTERN.scrollAtAbsoluteSpeed(MetersPerSecond.of(15),kLedSpacing);
    }

    public static Color returnRGBToGRB(double red, double green, double blue){
        return new Color(green,red,blue);
    }

    public LEDPattern returnRGBPattern(double red,double green,double blue){
        return LEDPattern.solid(returnRGBToGRB(red,green,blue));
    }
}
