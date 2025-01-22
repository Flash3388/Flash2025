package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
    private final AlgaeArm algaeArm;
    private final AlgaeGripper algaeGripper;
    private final CoralElevator coralElevator;
    private final CoralArm coralArm;
    private final CoralGripper coralGripper;

    public Dashboard(AlgaeArm algaeArm, AlgaeGripper algaeGripper, CoralElevator coralElevator, CoralArm coralArm, CoralGripper coralGripper) {
        this.algaeArm = algaeArm;
        this.algaeGripper = algaeGripper;
        this.coralElevator = coralElevator;
        this.coralArm = coralArm;
        this.coralGripper = coralGripper;
    }

    @Override
    public void periodic() {
        // Coral Gripper
        SmartDashboard.putBoolean("Coral Gripper Has Coral: ", coralGripper.hasCoral());
        SmartDashboard.putNumber("Coral Gripper Velocity: ", coralGripper.getVelocity());

        // Coral Elevator
        SmartDashboard.putBoolean("Coral Elevator Raised: " , coralElevator.isRaised());
        SmartDashboard.putBoolean("Coral Elevator Lowered: " , coralElevator.isLowered());

        // Coral Arm
        SmartDashboard.putBoolean("Coral Arm At Forward Limit: ", coralArm.isAtForwardLimit());
        SmartDashboard.putBoolean("Coral Arm At Reverse Limit: ", coralArm.isAtReverseLimit());
        SmartDashboard.putNumber("Coral Arm Degree: ", coralArm.getPositionDegrees());

        // Algae Gripper
        SmartDashboard.putBoolean("Algae Gripper Has Algae: ", algaeGripper.hasAlgae());
        SmartDashboard.putNumber("Algae Gripper Velocity: ", algaeGripper.getVelocity());

        // Algae Arm
        SmartDashboard.putBoolean("Algae Arm Is Extended: ", algaeArm.isExtended());
        SmartDashboard.putBoolean("Algae Arm Is Retracted: ", algaeArm.isRetracted());
    }

}
