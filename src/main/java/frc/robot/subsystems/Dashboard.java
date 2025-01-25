package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class Dashboard extends SubsystemBase {
    private final AlgaeArm algaeArm;
    private final AlgaeGripper algaeGripper;
    private final CoralElevator coralElevator;
    private final CoralArm coralArm;
    private final CoralGripper coralGripper;

    private final ShuffleboardTab coralTab;
    private final ShuffleboardTab algaeTab;

    public Dashboard(AlgaeArm algaeArm, AlgaeGripper algaeGripper, CoralElevator coralElevator, CoralArm coralArm, CoralGripper coralGripper) {
        this.algaeArm = algaeArm;
        this.algaeGripper = algaeGripper;
        this.coralElevator = coralElevator;
        this.coralArm = coralArm;
        this.coralGripper = coralGripper;

        coralTab = Shuffleboard.getTab("Coral Systems");
        algaeTab = Shuffleboard.getTab("Algae Systems");

        // Coral Gripper
        coralTab.addBoolean("Gripper Has Coral", coralGripper::hasCoral)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 0)
                .withSize(2, 1);

        coralTab.addNumber("Gripper Velocity", coralGripper::getVelocity)
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(2, 0)
                .withSize(4, 3);

        // Coral Elevator
        coralTab.addBoolean("Elevator Raised", coralElevator::isRaised)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 1)
                .withSize(2, 1);

        coralTab.addBoolean("Elevator Lowered", coralElevator::isLowered)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 2)
                .withSize(2, 1);

        // Coral Arm
        coralTab.addNumber("Arm Degree", coralArm::getPositionDegrees)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 180))
                .withPosition(6, 0)
                .withSize(2, 2);

        coralTab.addBoolean("Arm At FL", coralArm::isAtForwardLimit)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(6, 2)
                .withSize(1, 1);

        coralTab.addBoolean("Arm At RL", coralArm::isAtReverseLimit)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(7, 2)
                .withSize(1, 1);

        // Algae Gripper
        algaeTab.addBoolean("Gripper Has Algae", algaeGripper::hasAlgae)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 0)
                .withSize(2, 1);

        algaeTab.addNumber("Gripper Velocity", algaeGripper::getVelocity)
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(2, 0)
                .withSize(4, 3);

        // Algae Arm
        algaeTab.addBoolean("Arm Is Extended", algaeArm::isExtended)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 1)
                .withSize(2, 1);

        algaeTab.addBoolean("Arm Is Retracted", algaeArm::isRetracted)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 2)
                .withSize(2, 1);
    }
}
