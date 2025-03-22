package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
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

import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {

    private static final double WIDTH = 0.84;
    private static final double LENGTH = 0.85;
    private static final double MAX_SPEED = 4;

    private double lastXSpeed = 0;
    private double lastYSpeed = 0;
    private double lastRotation = 0;
    private static final double MAX_DELTA = 0.5;

    public final SwerveDrive swerveDrive;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d[] moduleMechanisms;

    public Swerve() {
        PIDFConfig drivePidf = new PIDFConfig(0.001153, 0, 0.5, 0, 0);
        PIDFConfig steerPidf = new PIDFConfig(0.012, 0, 0, 0, 0);
        ConversionFactorsJson conversionFactor = new ConversionFactorsJson();
        conversionFactor.drive.gearRatio = 6.75;
        conversionFactor.drive.factor = 0;
        conversionFactor.drive.diameter = 3.92 - 0.061;
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
                177.451172,
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
        Pigeon2Swerve pigeon = new Pigeon2Swerve(RobotMap.SWERVE_PIGEON);
        SwerveDriveConfiguration configuration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[] {
                        frontLeft, frontRight, backLeft, backRight
                },
                pigeon,
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
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();
        swerveDrive.setOdometryPeriod(0.004);

        for (SwerveModule module : swerveDrive.getModules()) {
            module.getAngleMotor().setMotorBrake(false);
        }

        swerveDrive.resetOdometry(new Pose2d(3,3,Rotation2d.kZero));

        PathPlannerLogging.setLogActivePathCallback((poses)-> {
            swerveDrive.field.getObject("trajectory").setPoses(poses);
        });

        setUpPathPlanner();

        mechanism = new Mechanism2d(50, 50);
        moduleMechanisms = createMechanismDisplay(mechanism);
        SmartDashboard.putData("SwerveMechanism", mechanism);
    }

    public Command driveA(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return runEnd(() -> {
            double xSpeed = MathUtil.applyDeadband(translationX.getAsDouble(),0.05);
            double ySpeed = MathUtil.applyDeadband(translationY.getAsDouble(),0.05);
            double rotation = MathUtil.applyDeadband(angularRotationX.getAsDouble(),0.05);

            if (Math.abs(xSpeed) < 0.02 && Math.abs(ySpeed) < 0.02 && Math.abs(rotation) < 0.02){
                stop();
                return;
            }

            xSpeed *= swerveDrive.getMaximumChassisVelocity();
            ySpeed *= swerveDrive.getMaximumChassisVelocity();
            rotation *= swerveDrive.getMaximumChassisAngularVelocity();

            double deltaX = xSpeed - lastXSpeed;
            if (Math.abs(deltaX) > MAX_DELTA){
                xSpeed = lastXSpeed + Math.signum(deltaX) * MAX_DELTA;
            }
            double deltaY = ySpeed - lastYSpeed;
            if (Math.abs(deltaY) > MAX_DELTA){
                ySpeed = lastYSpeed + Math.signum(deltaY) * MAX_DELTA;
            }

            double deltaRot = rotation - lastRotation;
            if (Math.abs(deltaRot) > MAX_DELTA) {
                rotation = lastRotation + Math.signum(deltaRot) * MAX_DELTA;
            }
            lastXSpeed = xSpeed;
            lastYSpeed = ySpeed;
            lastRotation = rotation;

            xSpeed = MathUtil.clamp(xSpeed,-3.5,3.5);
            ySpeed = MathUtil.clamp(ySpeed,-3.5,3.5);
            rotation = MathUtil.clamp(rotation,-Math.PI,Math.PI);

            swerveDrive.drive(
                    SwerveMath.scaleTranslation(new Translation2d(xSpeed, ySpeed),0.8),
                    rotation,
                    true,
                    false
            );
        },this::stop);
    }

    public Command driveSmall(double distanceMeters) {
        return new Command() {

            Pose2d startPose;

            {
                addRequirements(Swerve.this);
            }

            @Override
            public void initialize() {
                for (SwerveModule module : swerveDrive.getModules()) {
                    module.setAngle(0);
                }

                startPose = swerveDrive.getPose();

                System.out.println("Starting");
            }

            @Override
            public void execute() {
                for (SwerveModule module : swerveDrive.getModules()) {
                    module.getDriveMotor().set(0.1);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
                System.out.println("Done");
            }

            @Override
            public boolean isFinished() {
                Pose2d pose = swerveDrive.getPose();
                return pose.getTranslation().getDistance(startPose.getTranslation()) >= distanceMeters;
            }
        };
    }
    
    public void stop(){
            for (SwerveModule module : swerveDrive.getModules()) {
                module.getDriveMotor().set(0);
                module.getAngleMotor().set(0);
            }
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        int i =0;
        for(SwerveModule module: swerveDrive.getModules()){
            swerveModulePositions[i] = module.getPosition();
        }
        return swerveModulePositions;
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void updatePoseEstimator(LimelightHelpers.PoseEstimate poseEstimate){
        swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    @Override
    public void periodic() {
        SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
        for (int i = 0; i < modulePositions.length; i++) {
            moduleMechanisms[i].setAngle(modulePositions[i].angle.getDegrees() + 90);
        }
    }

    private void setUpPathPlanner(){
        RobotConfig config;
        try {
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
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            );
        } catch (Exception e) {
            throw new Error(e);
        }
    }

    private void setSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            stop();
        } else {
            swerveDrive.setChassisSpeeds(speeds);
        }
    }

    public Field2d getField() {
        return swerveDrive.field;
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
