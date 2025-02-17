package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LedLights extends SubsystemBase {
    private static final int kPort = RobotMap.LEDS_ID;
    private static final int kLength = RobotMap.LEDS_LENGTH * RobotMap.LEDS_PER_METER;
    private static final Distance kLedSpacing = Meters.of((double) RobotMap.LEDS_LENGTH /RobotMap.LEDS_PER_METER);

    private static final LEDPattern rainbowPattern = LEDPattern.rainbow(200, 100);
    private static final LEDPattern movingRainbow = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(5), kLedSpacing);

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    public LedLights() {
        leds = new AddressableLED(kPort);
        ledBuffer = new AddressableLEDBuffer(kLength);
        leds.setLength(kLength);
        leds.start();

    }

    @Override
    public void periodic() {
        leds.setData(ledBuffer);
    }

    public LEDPattern movingRainbow() {
        return movingRainbow;
    }

    public Command runPattern(LEDPattern pattern) {
        return runOnce(() -> pattern.applyTo(ledBuffer));
    }
}