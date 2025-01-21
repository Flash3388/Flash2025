package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CoralGripper;

public class Robot extends TimedRobot {
    private CoralGripper coralGripper;


    @Override
    public void robotInit() {
        coralGripper = new CoralGripper();


    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}
