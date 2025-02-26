// RobotContainer.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class RobotContainer {
    // Maximum speeds obtained from TunerConstants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //(m/s)
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); //(rad/s)
    SparkMax ceyhun = new SparkMax(1, MotorType.kBrushless);

    // =========== P I D =========== (FOR TURNING LMAOOOOOOOOOOOOOOOOOOOOO)
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.01;
    private static final double TOLERANCE = 1.0; //degs
    private final PIDController autoCenterPID = new PIDController(kP, kI, kD);

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
        autoCenterPID.setTolerance(TOLERANCE);
        configureBindings();
    }

    private void configureBindings() {

     

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -joystick.getLeftY() * MaxSpeed;
                double velocityY = joystick.getLeftX() * MaxSpeed;
                
                double rotationalRate = -joystick.getRightX() * MaxAngularRate;
                
                if (joystick.button(3).getAsBoolean()) { //might arise problems lmao TODO: change button number lol
                    if (LimelightHelpers.getTV("maylimelight")) {
                        double tx = LimelightHelpers.getTX("maylimelight"); // tx in degs
                        rotationalRate = autoCenterPID.calculate(tx, 0.0);
                    } else {
                        autoCenterPID.reset();
                        rotationalRate = 0.0;
                    }
                } else {
                    autoCenterPID.reset();
                }
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
        return Commands.print("No autonomous command configured");
    }
}
