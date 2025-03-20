package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX m_climberMotor;

    public ClimberSubsystem() {
        m_climberMotor = new TalonFX(ClimberSubsystemConstants.kClimberMotorCanId);

        // Set neutral mode to brake to hold the climber position when motor power is not applied
        m_climberMotor.setNeutralMode(NeutralModeValue.Brake);

        // Set the safety enabled to ensure the motor stops if not regularly updated
        m_climberMotor.setSafetyEnabled(true);
        m_climberMotor.setExpiration(0.1); // 100ms expiration time for the safety feature
    }

    @Override
    public void periodic() {
        // Feed the motor safety object to ensure the motor does not get disabled
        m_climberMotor.feed();

        // Display the climber encoder position and velocity on the SmartDashboard
        SmartDashboard.putNumber("Climber Encoder Position", getClimberEncoderPosition());
        SmartDashboard.putNumber("Climber Encoder Velocity", getClimberEncoderVelocity());
    }

    public void setClimberMotor(double speed) {
        m_climberMotor.set(speed);
    }

    public Command moveClimber(double speed) {
        return new MoveClimberCommand(speed);
    }

    // Method to get the current position of the climber encoder
    public double getClimberEncoderPosition() {
        return m_climberMotor.getPosition().getValueAsDouble();
    }

    // Method to get the current velocity of the climber encoder
    public double getClimberEncoderVelocity() {
        return m_climberMotor.getVelocity().getValueAsDouble();
    }

    // Inner class for operating the climber
    public class MoveClimberCommand extends Command {
        private final double speed;

        public MoveClimberCommand(double speed) {
            this.speed = speed;
            // Add requirements to ensure this command has exclusive access to the ClimberSubsystem
            addRequirements(ClimberSubsystem.this);
        }

        @Override
        public void execute() {
            // Set the climber motor speed
            setClimberMotor(speed);
        }

        @Override
        public void end(boolean interrupted) {
            // Command end action: Stop the climber motor, whether the command ends normally or is interrupted
            setClimberMotor(0);
        }
    }
}