package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotMap;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.motors.SparkFlexSwerve;
import swervelib.parser.*;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.telemetry.SwerveDriveTelemetry;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {

    private static final double WIDTH = 0.707;
    private static final double LENGTH = 0.702;
    private static final double MAX_SPEED = 4;
    private final SwerveDrive swerveDrive;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d[] moduleMechanisms;


    public Swerve() {
        PIDFConfig drivePidf = new PIDFConfig(0.001153, 0, 0.50, 0, 0);
        PIDFConfig steerPidf = new PIDFConfig(0.01, 0, 0, 0, 0);
        ConversionFactorsJson conversionFactor = new ConversionFactorsJson();
        conversionFactor.drive.gearRatio = 6.75;
        conversionFactor.drive.factor = 0;
        conversionFactor.drive.diameter = 2 * 2;
        conversionFactor.angle.gearRatio = 12.8;
        conversionFactor.angle.factor = 0;

        conversionFactor.drive.calculate();
        conversionFactor.angle.calculate();

        SwerveModulePhysicalCharacteristics characteristics = new SwerveModulePhysicalCharacteristics(conversionFactor,0.25,0.25);
        SwerveModuleConfiguration frontLeft = new SwerveModuleConfiguration(
                new SparkFlexSwerve(RobotMap.SWERVE_FRONT_LEFT_DRIVE,true,DCMotor.getNeoVortex(1)),
                new SparkFlexSwerve(RobotMap.SWERVE_FRONT_LEFT_STEER,false,DCMotor.getNeoVortex(1)),
                conversionFactor,
                new CANCoderSwerve(RobotMap.SWERVE_FRONT_LEFT_ENCODER),
                315,
                LENGTH / 2,
                WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                false,
                false,
                "FrontLeft",
                false
        );
        SwerveModuleConfiguration frontRight = new SwerveModuleConfiguration(
                new SparkFlexSwerve(RobotMap.SWERVE_FRONT_RIGHT_DRIVE,true,DCMotor.getNeoVortex(1)),
                new SparkFlexSwerve(RobotMap.SWERVE_FRONT_RIGHT_STEER,false,DCMotor.getNeoVortex(1)),
                conversionFactor,
                new CANCoderSwerve(RobotMap.SWERVE_FRONT_RIGHT_ENCODER),
                281,
                LENGTH / 2,
                -WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                false,
                false,
                "FrontRight",
                false
        );
        SwerveModuleConfiguration backLeft = new SwerveModuleConfiguration(
                new SparkFlexSwerve(RobotMap.SWERVE_BACK_LEFT_DRIVE,true,DCMotor.getNeoVortex(1)),
                new SparkFlexSwerve(RobotMap.SWERVE_BACK_LEFT_STEER,false,DCMotor.getNeoVortex(1)),
                conversionFactor,
                new CANCoderSwerve(RobotMap.SWERVE_BACK_LEFT_ENCODER),
                256.2,
                -LENGTH / 2,
                WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                false,
                false,
                "BackLeft",
                false
        );
        SwerveModuleConfiguration backRight = new SwerveModuleConfiguration(
                new SparkFlexSwerve(RobotMap.SWERVE_BACK_RIGHT_DRIVE,true,DCMotor.getNeoVortex(1)),
                new SparkFlexSwerve(RobotMap.SWERVE_BACK_RIGHT_STEER,false,DCMotor.getNeoVortex(1)),
                conversionFactor,
                new CANCoderSwerve(RobotMap.SWERVE_BACK_RIGHT_ENCODER),
                181.5,
                -LENGTH / 2,
                -WIDTH / 2,
                steerPidf,
                drivePidf,
                characteristics,
                false,
                false,
                false,
                "BackRight",
                false
        );
        SwerveDriveConfiguration configuration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[] {
                        frontLeft, frontRight, backLeft, backRight
                },
                new Pigeon2Swerve(RobotMap.SWERVE_PIGEON),
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

        swerveDrive = new SwerveDrive(configuration, controllerConfiguration, MAX_SPEED, Pose2d.kZero);
        swerveDrive.setHeadingCorrection(false); // TODO : try running with heading correction.
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();

        swerveDrive.resetOdometry(Pose2d.kZero);

        mechanism = new Mechanism2d(50, 50);
        moduleMechanisms = createMechanismDisplay(mechanism);
        SmartDashboard.putData("SwerveMechanism", mechanism);

        PathPlannerLogging.setLogActivePathCallback((poses)-> {
            swerveDrive.field.getObject("trajectory").setPoses(poses);
        });

        setUpPathPlanner();
    }

    public Command driveA(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return runEnd(() -> {
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    false,
                    false);
        },this::stop);
    }

    public void rotate(DoubleSupplier rotationX){
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                        0,
                        0), 0.8),
                rotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                false,
                false);
    }
    
    public void stop(){
            for (SwerveModule module : swerveDrive.getModules()) {
                module.getDriveMotor().set(0);
                module.getAngleMotor().set(0);
            }
    }

    public PathPlannerPath alignToReefLeft(VisionSystem visionSystem){
        List<Waypoint> waypoints;
        PathPlannerPath path;

        int id = visionSystem.frontGetTargetId();
        Pose2d leftPose = visionSystem.getMovingPoseLeft(id);

        rotate(()-> visionSystem.getMovingAngle(id));

        waypoints = PathPlannerPath.waypointsFromPoses(
                getPose(),
                leftPose
        );
        PathConstraints constraints = new PathConstraints(0.5,0.5,Math.PI*2,Math.PI);

        path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0,new Rotation2d(visionSystem.getMovingAngle(id)))
        );
        path.preventFlipping = true;
        return path;
    }

    public boolean isNear(Pose2d otherPose) {
        double tolerance = 0.05;
        Pose2d robotPose = getPose();
        double deltaX = Math.abs(robotPose.getX() - otherPose.getX());
        double deltaY = Math.abs(robotPose.getY() - otherPose.getY());

        return deltaX <= tolerance && deltaY <= tolerance;
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
        swerveDrive.resetOdometry(Pose2d.kZero);
    }

    public void setUpPathPlanner(){
        RobotConfig config;
        try
        {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    this::setSpeeds,
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
            throw new Error(e);
        }
        PathfindingCommand.warmupCommand().schedule();
    }

    public ChassisSpeeds getSpeeds(){
        return swerveDrive.getRobotVelocity();
    }

    public Command centerModules() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }


    @Override
    public void periodic() {
        SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
        swerveDrive.updateOdometry();

        for (int i = 0; i < modulePositions.length; i++) {
            moduleMechanisms[i].setAngle(modulePositions[i].angle.getDegrees() + 90);
        }

    }
    public void updatePoseEstimator(LimelightHelpers.PoseEstimate poseEstimate){
            swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);

    }


    private void setSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            for (SwerveModule module : swerveDrive.getModules()) {
                module.getDriveMotor().set(0);
                module.getAngleMotor().set(0);
            }
        } else {
            swerveDrive.setChassisSpeeds(speeds);
        }
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

}
