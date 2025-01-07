package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import swervelib.SwerveDrive;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.motors.SparkMaxSwerve;
import swervelib.parser.*;
import swervelib.parser.json.MotorConfigDouble;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {

    private static final double WIDTH = 0.707;
    private static final double LENGTH = 0.702;
    private static final double MAX_SPEED = 4;
    private final SwerveDrive swerveDrive;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d[] moduleMechanisms;

    public Swerve() {
        PIDFConfig drivePidf = new PIDFConfig(0.0020645, 0, 0, 0, 0);
        PIDFConfig steerPidf = new PIDFConfig(0.01, 0, 0, 0, 0);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(0.0508, 6.75);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        ConversionFactorsJson conversionFactor = new ConversionFactorsJson();
        SwerveModulePhysicalCharacteristics characteristics = new SwerveModulePhysicalCharacteristics(conversionFactor,0.25,0.25);

        SwerveModuleConfiguration frontLeft = new SwerveModuleConfiguration(
                new SparkMaxSwerve(31, true, DCMotor.getNEO(1)),
                new SparkMaxSwerve(32, false,DCMotor.getNEO(1)),
                conversionFactor,
                new CANCoderSwerve(3),
                15,
                LENGTH / 2,
                WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                true,
                false,
                "FrontLeft",
                false
        );
        SwerveModuleConfiguration frontRight = new SwerveModuleConfiguration(
                new SparkMaxSwerve(52, true,DCMotor.getNEO(1)),
                new SparkMaxSwerve(18, false,DCMotor.getNEO(1)),
                conversionFactor,
                new CANCoderSwerve(4),
                254.3,
                LENGTH / 2,
                -WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                true,
                false,
                "FrontRight",
                false
        );
        SwerveModuleConfiguration backLeft = new SwerveModuleConfiguration(
                new SparkMaxSwerve(61, true,DCMotor.getNEO(1)),
                new SparkMaxSwerve(12, false,DCMotor.getNEO(1)),
                conversionFactor,
                new CANCoderSwerve(6),
                44.2,
                -LENGTH / 2,
                WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                true,
                false,
                "BackLeft",
                false
        );
        SwerveModuleConfiguration backRight = new SwerveModuleConfiguration(
                new SparkMaxSwerve(13, true,DCMotor.getNEO(1)),
                new SparkMaxSwerve(42, false,DCMotor.getNEO(1)),
                conversionFactor,
                new CANCoderSwerve(5),
                358.33,
                -LENGTH / 2,
                -WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                true,
                false,
                "BackRight",
                false
        );
        SwerveDriveConfiguration configuration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[] {
                        frontLeft, frontRight, backLeft, backRight
                },
                new Pigeon2Swerve(11),
                false,
                characteristics
        );
        SwerveControllerConfiguration controllerConfiguration = new SwerveControllerConfiguration(
                configuration,
                new PIDFConfig(0.4, 0, 0.01),
                0.5,
                MAX_SPEED
        );

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        swerveDrive = new SwerveDrive(configuration, controllerConfiguration, MAX_SPEED,new Pose2d());// dont know what to do with the pose currntly
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();

        mechanism = new Mechanism2d(50, 50);
        moduleMechanisms = createMechanismDisplay(mechanism);
        SmartDashboard.putData("SwerveMechanism", mechanism);

        PathPlannerLogging.setLogActivePathCallback((poses)-> {
            swerveDrive.field.getObject("trajectory").setPoses(poses);
        });
    }

    public Command driveA(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    false,
                    false);
        });
    }

    public Pose2d getPose()
    {

        return swerveDrive.getPose();
    }
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }
    public void resetOdometeryToStart() {
        swerveDrive.swerveDrivePoseEstimator.resetPosition(
                swerveDrive.getOdometryHeading(),
                swerveDrive.getModulePositions(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        );
    }
    public void setUpPathPlanner(){
        RobotConfig config;
        try
        {
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedforward = true;
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward)
                        {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces()
                            );
                        }
                        else
                        {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
                    ),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent())
                        {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            );
        } catch (Exception e)
        {
            e.printStackTrace();
        }
        PathfindingCommand.warmupCommand().schedule();
    }
    public Command getAutonomousCommand(String pathName)
    {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto(pathName);
    }
    public ChassisSpeeds getSpeeds(){
        return swerveDrive.getRobotVelocity();
    }
    public void drive(ChassisSpeeds speeds){
        swerveDrive.drive(speeds);
    }
    public Command centerModules() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }


    @Override
    public void periodic() {
        SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
        for (int i = 0; i < modulePositions.length; i++) {
            moduleMechanisms[i].setAngle(modulePositions[i].angle.getDegrees() + 90);



        }
    }
    public void resetPose(Pose2d pose){
        swerveDrive.resetOdometry(pose);
    }

    private MechanismLigament2d[] createMechanismDisplay(Mechanism2d mechanism) {
        final double BOTTOM = 5;
        final double TOP = 45;
        final double BEAM_LENGTH = 40;
        final double BEAM_WIDTH = 2;
        final double WHEEL_DIR_LENGTH = 5;
        final double WHEEL_DIR_WIDTH = 10;
        final Color8Bit BEAM_COLOR = new Color8Bit(0, 254, 52);
        final Color8Bit WHEEL_DIR_COLOR = new Color8Bit(255, 0, 0);

        MechanismRoot2d driveBaseMechanismBottomLeft = mechanism.getRoot("drivebase-bottomleft", BOTTOM, BOTTOM);
        MechanismRoot2d driveBaseMechanismTopRight = mechanism.getRoot("drivebase-topright", TOP, TOP);
        MechanismRoot2d driveBaseMechanismBottomRight = mechanism.getRoot("drivebase-bottomright", TOP, BOTTOM);
        MechanismRoot2d driveBaseMechanismTopLeft = mechanism.getRoot("drivebase-topleft", BOTTOM, TOP);

        driveBaseMechanismBottomLeft.append(new MechanismLigament2d("bottom", BEAM_LENGTH, 0, BEAM_WIDTH, BEAM_COLOR));
        driveBaseMechanismBottomLeft.append(new MechanismLigament2d("left", BEAM_LENGTH, 90, BEAM_WIDTH, BEAM_COLOR));
        driveBaseMechanismTopRight.append(new MechanismLigament2d("top", BEAM_LENGTH, 180, BEAM_WIDTH, BEAM_COLOR));
        driveBaseMechanismTopRight.append(new MechanismLigament2d("right", BEAM_LENGTH, 270, BEAM_WIDTH, BEAM_COLOR));

        MechanismLigament2d mechanismBottomLeft = driveBaseMechanismBottomLeft.append(new MechanismLigament2d("module-bottomleft", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));
        MechanismLigament2d mechanismBottomRight = driveBaseMechanismBottomRight.append(new MechanismLigament2d("module-bottomright", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));
        MechanismLigament2d mechanismTopLeft = driveBaseMechanismTopLeft.append(new MechanismLigament2d("module-topleft", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));
        MechanismLigament2d mechanismTopRight = driveBaseMechanismTopRight.append(new MechanismLigament2d("module-topright", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));

        return new MechanismLigament2d[] {
                mechanismTopLeft,
                mechanismTopRight,
                mechanismBottomLeft,
                mechanismBottomRight
        };
    }
    public void updatePoseEstimator(Pose2d pos, double time){
        swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(pos,time);
    }
}
