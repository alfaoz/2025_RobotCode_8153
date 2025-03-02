// RobotContainer.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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


    public RobotContainer() {
        CommandRollerSpew rollerSpew = new CommandRollerSpew(ceyhun);
        CommandRollerReverse rollerReverse = new CommandRollerReverse(ceyhun);
        AutonRollerReverseCommand autonRollerReverse = new AutonRollerReverseCommand(ceyhun);
        AutonRollerSpewCommand autonRollerSpew = new AutonRollerSpewCommand(ceyhun);

        NamedCommands.registerCommand("autonRollerReverse", autonRollerReverse);
        NamedCommands.registerCommand("autonRollerSpew", autonRollerSpew);


        configureBindings(rollerReverse, rollerSpew);
        
        // Build an auto chooser with all autos in the project
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings(CommandRollerReverse rollerReverse, CommandRollerSpew rollerSpew) {
        // Create the AprilTag alignment command
        AlignWithAprilTagCommand alignWithAprilTagCommand = 
            new AlignWithAprilTagCommand(drivetrain, LIMELIGHT_NAME, CLOSE_ENOUGH_DISTANCE, drive, joystick, MaxSpeed);
            
        
        
        // Create the Limelight lights toggle command
        ToggleLimelightLightsCommand toggleLimelightLightsCommand = new ToggleLimelightLightsCommand(LIMELIGHT_NAME);
        // Add a description to the SmartDashboard
        SmartDashboard.putString("AprilTag/Info", "Press Button 3 to align with AprilTag");

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -0.8 *Math.pow(joystick.getLeftY(),3) * MaxSpeed;
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
        joystick.button(5).whileTrue(alignWithAprilTagCommand);
        joystick.button(3).whileTrue(rollerSpew);
        joystick.button(4).whileTrue(rollerReverse);
        
        // Bind the Limelight lights command to the left trigger with a very low threshold (5%)
        // Using whileTrue() to keep lights on while trigger is held (like a car flasher)
        joystick.leftTrigger(0.8).whileTrue(toggleLimelightLightsCommand);
        
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
        
        // Alternatively, you can return a specific auto:
        // return new PathPlannerAuto("Example Auto");
    }
}
