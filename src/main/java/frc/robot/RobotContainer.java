// RobotContainer.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignWithAprilTagCommand;
import frc.robot.commands.AutonRollerReverseCommand;
import frc.robot.commands.AutonRollerSpewCommand;
import frc.robot.commands.CommandRollerReverse;
import frc.robot.commands.CommandRollerSpew;
import frc.robot.commands.LimelightPoseEstimatorCommand;
import frc.robot.commands.ToggleLimelightLightsCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class RobotContainer {
    // Auto chooser for selecting autonomous routines
    private final SendableChooser<Command> autoChooser;
    // Maximum speeds obtained from TunerConstants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //(m/s)
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); //(rad/s)
    SparkMax ceyhun = new SparkMax(1, MotorType.kBrushless);


    //====== APRIL TAG ALIGNMENT CONSTANTS ============
    private static final double CLOSE_ENOUGH_DISTANCE = 1.0; // metres
    // Default Limelight name is "limelight" unless changed in Limelight config
    private static final String LIMELIGHT_NAME = "limelight";

    /* Swerve drive request for field-centric control */ // PID AUTOCENTER MIGHT BREAK!!!!!!!!!!!!!!!!
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.05) //10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //controller for driver inp
    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Shuffleboard entries for Limelight data
    private GenericEntry m_validPoseEntry;
    private GenericEntry m_poseXEntry;
    private GenericEntry m_poseYEntry;
    private GenericEntry m_poseRotEntry;
    private GenericEntry m_tagCountEntry;
    
    // Limelight pose estimator command
    private LimelightPoseEstimatorCommand m_continuousLimelightPoseEstimator;


    public RobotContainer() throws FileVersionException, IOException, ParseException {
        CommandRollerSpew rollerSpew = new CommandRollerSpew(ceyhun);
        CommandRollerReverse rollerReverse = new CommandRollerReverse(ceyhun);
        AutonRollerReverseCommand autonRollerReverse = new AutonRollerReverseCommand(ceyhun);
        AutonRollerSpewCommand autonRollerSpew = new AutonRollerSpewCommand(ceyhun);

        // Create Shuffleboard widgets for Limelight data
        ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
        
        m_validPoseEntry = limelightTab.add("Valid Pose", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(2, 1)
            .withPosition(0, 0)
            .getEntry();
        
        m_poseXEntry = limelightTab.add("X Position", 0.0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();
        
        m_poseYEntry = limelightTab.add("Y Position", 0.0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();
        
        m_poseRotEntry = limelightTab.add("Rotation", 0.0)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();
        
        m_tagCountEntry = limelightTab.add("Tag Count", 0)
            .withPosition(3, 1)
            .withSize(1, 1)
            .getEntry();
        
        // Create the continuous Limelight pose estimator command as a default command
        // This will run continuously and update the robot's position whenever the Limelight can see AprilTags
        m_continuousLimelightPoseEstimator = 
            new LimelightPoseEstimatorCommand(drivetrain, LIMELIGHT_NAME, logger);
        
        // Set the Shuffleboard entries for the continuous pose estimator
        m_continuousLimelightPoseEstimator.setShuffleboardEntries(
            m_validPoseEntry,
            m_poseXEntry,
            m_poseYEntry,
            m_poseRotEntry,
            m_tagCountEntry
        );
        
        // Set up the continuous pose estimator as a default command for the drivetrain
        // This ensures it runs continuously and doesn't get interrupted by other commands
        // We don't require the drivetrain subsystem to avoid conflicts with other commands
        Command continuousPoseCommand = m_continuousLimelightPoseEstimator.ignoringDisable(true);
        continuousPoseCommand.schedule();
        
        // Also register it to run periodically in Robot.java's robotPeriodic method
        SmartDashboard.putData("Continuous Limelight Pose Estimator", m_continuousLimelightPoseEstimator);
        
        // Print debug info to confirm it's running
        System.out.println("Continuous Limelight pose estimator scheduled");
        
        // Add information to SmartDashboard
        SmartDashboard.putString("Limelight/Info", "Limelight pose estimation active");
        
        // Create a boolean indicator for pose validity on SmartDashboard
        // This will be updated by the LimelightPoseEstimatorCommand
        SmartDashboard.putBoolean("LIMELIGHT_VALID_POSE_INDICATOR", false);

        NamedCommands.registerCommand("autonRollerReverse", autonRollerReverse);
        NamedCommands.registerCommand("autonRollerSpew", autonRollerSpew);

        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);

        // pathFiles
        PathPlannerPath alftRsn_Path = PathPlannerPath.fromPathFile("alft-rsn");
        PathPlannerPath alftLsn_Path = PathPlannerPath.fromPathFile("alft-lsn");


        // pathConstraints
        PathConstraints alftRsn_Constraints = new PathConstraints(5.0, 2.5,Units.degreesToRadians(200), Units.degreesToRadians(200));
        PathConstraints alftLsn_Constraints = new PathConstraints(5.0, 2.5,Units.degreesToRadians(200), Units.degreesToRadians(200));


        // Create pathfinding commands
        // Since we have the continuous pose estimator running, we don't need to
        // run a separate pose update before pathfinding
        Command alftRsn_Command = AutoBuilder.pathfindThenFollowPath(alftRsn_Path, alftRsn_Constraints);
        Command alftLsn_Command = AutoBuilder.pathfindThenFollowPath(alftLsn_Path, alftLsn_Constraints);

        // Create a separate instance for the button binding
        // This allows the driver to force a pose update at any time
        LimelightPoseEstimatorCommand buttonPoseEstimator = 
            new LimelightPoseEstimatorCommand(drivetrain, LIMELIGHT_NAME, logger);
        
        // Set the Shuffleboard entries for the button pose estimator
        buttonPoseEstimator.setShuffleboardEntries(
            m_validPoseEntry,
            m_poseXEntry,
            m_poseYEntry,
            m_poseRotEntry,
            m_tagCountEntry
        );

        configureBindings(rollerReverse, rollerSpew, alftRsn_Command, alftLsn_Command, buttonPoseEstimator);
    }

    private void configureBindings(CommandRollerReverse rollerReverse, CommandRollerSpew rollerSpew, 
                                  Command alftRsn_Command, Command alftLsn_Command, 
                                  LimelightPoseEstimatorCommand buttonPoseEstimator) {
        // Create the AprilTag alignment command
        AlignWithAprilTagCommand alignWithAprilTagCommand = 
            new AlignWithAprilTagCommand(drivetrain, LIMELIGHT_NAME, CLOSE_ENOUGH_DISTANCE, drive, joystick, MaxSpeed); 
        
        // Create the Limelight lights toggle command
        ToggleLimelightLightsCommand toggleLimelightLightsCommand = new ToggleLimelightLightsCommand(LIMELIGHT_NAME);
        // Add a description to the SmartDashboard
        SmartDashboard.putString("AprilTag/Info", "Press Button 3 to align with AprilTag");

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -0.8*Math.pow(joystick.getLeftY(),3) * MaxSpeed;
                double velocityY = -0.8*Math.pow(joystick.getLeftX(),3) * MaxSpeed;
                
                double rotationalRate = -0.1*joystick.getRightX()  * MaxAngularRate;
                
                // The AprilTag alignment is now handled by the AlignWithAprilTagCommand
                // when button 3 is pressed, so we don't need to check for it here
                
                
            
                return drive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
            })
        );
        
        // Bind the AprilTag alignment command to button 5
        joystick.button(6).whileTrue(alignWithAprilTagCommand);
        joystick.button(3).whileTrue(rollerSpew);
        joystick.button(4).whileTrue(rollerReverse);

        // Bind the path commands to D-pad buttons
        joystick.povRight().whileTrue(alftRsn_Command);
        joystick.povLeft().whileTrue(alftLsn_Command);
        
        // Bind the on-demand Limelight pose update to a button
        // This allows the driver to force a pose update at any time
        joystick.povUp().onTrue(buttonPoseEstimator.withTimeout(0.5));

        // Bind the Limelight lights toggle command
        joystick.leftTrigger(0.8).toggleOnTrue(toggleLimelightLightsCommand);
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Return the selected auto from the chooser
        return autoChooser.getSelected();
    }
}
