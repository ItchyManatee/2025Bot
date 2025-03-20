package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.CoralGroundIntakeSubsystemConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intakeMotorFx;
    private SparkMax m_pivotMotor =
      new SparkMax(CoralGroundIntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushed);

    private SparkMaxConfig m_pivotMotorConfig;
    private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(CoralGroundIntakeSubsystemConstants.kPivotEncoderPort);

    private SparkClosedLoopController m_pivot_ClosedLoopController = m_pivotMotor.getClosedLoopController();

    private final PIDController m_pivotPIDController = new PIDController(CoralGroundIntakeSubsystemConstants.kP, CoralGroundIntakeSubsystemConstants.kI, CoralGroundIntakeSubsystemConstants.kD);
    private final SimpleMotorFeedforward m_pivotFeedforward = new SimpleMotorFeedforward(CoralGroundIntakeSubsystemConstants.kS, CoralGroundIntakeSubsystemConstants.kV);

    // Define setpoints for Stow, Score, and Ground positions
    private static final double STOW_POSITION = 0.05;
    private static final double SCORE_POSITION = 0.5;
    private static final double GROUND_POSITION = 0.95;

    XboxController opController = new XboxController(1);

    public CoralGroundIntakeSubsystem() {
        m_intakeMotorFx = new TalonFX(CoralGroundIntakeSubsystemConstants.kIntakeMotorCanId);

        // Set neutral mode to brake to hold the intake position when motor power is not applied
        m_intakeMotorFx.setNeutralMode(NeutralModeValue.Brake);

        // Set the safety enabled to ensure the motor stops if not regularly updated
        m_intakeMotorFx.setSafetyEnabled(true);
        m_intakeMotorFx.setExpiration(0.1); // 100ms expiration time for the safety feature
        
        m_pivotMotor.configure(
            Configs.CoralGroundIntakeSubsystem.pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Feed the motor safety object to ensure the motor does not get disabled
        m_intakeMotorFx.feed();

        // Display the pivot encoder position and connection status on the SmartDashboard
        SmartDashboard.putNumber("Pivot Encoder Position", getPivotEncoderPosition());
        SmartDashboard.putBoolean("Pivot Encoder Connected", isPivotEncoderConnected());

        // POV of operator controller
        //SmartDashboard.putNumber("POV Angle", opController.getPOV());
    }

    public void setIntakeMotor(double speed) {
        m_intakeMotorFx.set(speed);
    }

    public void setPivotMotor(double speed) {
        m_pivotMotor.set(speed);
    }

    public Command moveIntake(double speed) {
        return new MoveIntakeCommand(speed);
    }

    public Command movePivot(double speed) {
        return new MovePivotCommand(speed);
    }

    // Method to get the current position of the pivot encoder
    public double getPivotEncoderPosition() {
        return m_pivotEncoder.get();
    }

    // Method to check if the pivot encoder is connected
    public boolean isPivotEncoderConnected() {
        return m_pivotEncoder.isConnected();
    }

    // Command to move pivot to a specific position using PID control
    public Command movePivotToPosition(double setpoint) {
        return new MovePivotToPositionCommand(setpoint);
    }

    // Commands for moving to specific setpoints
    public Command moveToStowPosition() {
        return this.run(
        () -> {
            movePivotToPosition(STOW_POSITION);
        });
    }

    public Command moveToScorePosition() {
        return this.run(
        () -> {
            movePivotToPosition(SCORE_POSITION);
        });
    }

    public Command moveToGroundPosition(){
        return this.run(
        () -> {
            movePivotToPosition(GROUND_POSITION);
        });
    }

    public Command WhatPOVAngle(int angle) {
        SmartDashboard.putNumber("POV Angle", angle);
        return new Command() {
            // do nothing
        };
    }

    // Inner class for operating the intake
    public class MoveIntakeCommand extends Command {
        private final double speed;

        public MoveIntakeCommand(double speed) {
            this.speed = speed;
            // Add requirements to ensure this command has exclusive access to the CoralGroundIntakeSubsystem
            addRequirements(CoralGroundIntakeSubsystem.this);
        }

        @Override
        public void execute() {
            // Set the intake motor speed
            setIntakeMotor(speed);
        }

        @Override
        public void end(boolean interrupted) {
            // Command end action: Stop the intake motor, whether the command ends normally or is interrupted
            setIntakeMotor(0);
        }
    }

    // Inner class for operating the pivot
    public class MovePivotCommand extends Command {
        private final double speed;

        public MovePivotCommand(double speed) {
            this.speed = speed;
            // Add requirements to ensure this command has exclusive access to the CoralGroundIntakeSubsystem
            addRequirements(CoralGroundIntakeSubsystem.this);
        }

        @Override
        public void execute() {
            // Set the pivot motor speed
            setPivotMotor(speed);
        }

        @Override
        public void end(boolean interrupted) {
            // Command end action: Stop the pivot motor, whether the command ends normally or is interrupted
            setPivotMotor(0);
        }
    }

    // Command to move pivot to a specific position using PID control
    public class MovePivotToPositionCommand extends Command {
        private final double setpoint;

        public MovePivotToPositionCommand(double setpoint) {
            this.setpoint = setpoint;
            // Smart dashboard of what setpoint
            SmartDashboard.putNumber("Pivot Arm Setpoint", setpoint);
            // Add requirements to ensure this command has exclusive access to the CoralGroundIntakeSubsystem
            addRequirements(CoralGroundIntakeSubsystem.this);
        }

        @Override
        public void execute() {
            // if (!isPivotEncoderConnected()) {
            //     // If the encoder is not connected, stop the pivot motor and end the command
            //     setPivotMotor(0);
            //     cancel();
            //     return;
            // }

            SmartDashboard.putString("POS", "WHERE AM I");
            double pidOutput = m_pivotPIDController.calculate(getPivotEncoderPosition(), setpoint);
            double feedforwardOutput = m_pivotFeedforward.calculate(setpoint);
            setPivotMotor(pidOutput + feedforwardOutput);
        }

        @Override
        public void end(boolean interrupted) {
            // Command end action: Stop the pivot motor, whether the command ends normally or is interrupted
            setPivotMotor(0);
        }

        @Override
        public boolean isFinished() {
            return m_pivotPIDController.atSetpoint();
        }
    }

    // Idle command which stops both the pivot and intake motors
    public Command idleCommand() {
        return new IdleCommand();
    }

    // Inner class for idling the intake and pivot motors
    public class IdleCommand extends Command {
        public IdleCommand() {
            // Add requirements to ensure this command has exclusive access to the CoralGroundIntakeSubsystem
            addRequirements(CoralGroundIntakeSubsystem.this);
        }

        @Override
        public void initialize() {
            // Stop the intake motor
            setIntakeMotor(0);
            // Stop the pivot motor
            setPivotMotor(0);
        }

        @Override
        public void end(boolean interrupted) {
            // Command end action: Stop the intake and pivot motors, whether the command ends normally or is interrupted
            setIntakeMotor(0);
            setPivotMotor(0);
        }

        @Override
        public boolean isFinished() {
            // This command is never finished, it is the default command for the CoralGroundIntakeSubsystem
            return false;
        }
    }
}