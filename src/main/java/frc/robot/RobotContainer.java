// RobotContainer.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
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
    private static final String LIMELIGHT_NAME = "maylimelight";

    /* Swerve drive request for field-centric control */ // PID AUTOCENTER MIGHT BREAK!!!!!!!!!!!!!!!!
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) //10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //controller for driver inp
    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        
        // Build an auto chooser with all autos in the project
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Create the AprilTag alignment command
        AlignWithAprilTagCommand alignWithAprilTagCommand = 
            new AlignWithAprilTagCommand(drivetrain, LIMELIGHT_NAME, CLOSE_ENOUGH_DISTANCE, drive, joystick, MaxSpeed);
            
        // Add a description to the SmartDashboard
        SmartDashboard.putString("AprilTag/Info", "Press Button 3 to align with AprilTag");

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -joystick.getLeftY() * MaxSpeed;
                double velocityY = joystick.getLeftX() * MaxSpeed;
                
                double rotationalRate = -joystick.getRightX() * MaxAngularRate;
                
                // The AprilTag alignment is now handled by the AlignWithAprilTagCommand
                // when button 3 is pressed, so we don't need to check for it here
                
                if(joystick.button(8).getAsBoolean()){
                }
                else if(joystick.button(9).getAsBoolean()){

                }
                else{

                }
            
                return drive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
            })
        );
        
        // Bind the AprilTag alignment command to button 3
        joystick.button(3).whileTrue(alignWithAprilTagCommand);
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Shooter activation: right bumper runs the shooter motor while held
        joystick.rightBumper().whileTrue(Commands.startEnd(
            () -> ceyhun.set(1.0),
            () -> ceyhun.set(0.0)
        ));
        SmartDashboard.putString("Shooter/Info", "Press Right Bumper to activate shooter");

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Return the selected auto from the chooser
        return autoChooser.getSelected();
        
        // Alternatively, you can return a specific auto:
        // return new PathPlannerAuto("Example Auto");
    }
}
