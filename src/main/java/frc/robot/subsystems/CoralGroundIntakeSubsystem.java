package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.CoralGroundIntakeSubsystemConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intakeMotorFx;
    private final SparkMax m_pivotMotor;
    private final DutyCycleEncoder m_pivotEncoder;
    private final PIDController m_pivotPIDController;
    private final SimpleMotorFeedforward m_pivotFeedforward;

    private static final double STOW_POSITION = 0.400; // 0.414
    private static final double SCORE_POSITION = 0.285;
    private static final double GROUND_POSITION = 0.025;
    private static final double CATCH_POSITION = 0.055;

    public CoralGroundIntakeSubsystem() {
        m_intakeMotorFx = new TalonFX(CoralGroundIntakeSubsystemConstants.kIntakeMotorCanId);
        m_pivotMotor = new SparkMax(CoralGroundIntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushed);
        m_pivotEncoder = new DutyCycleEncoder(CoralGroundIntakeSubsystemConstants.kPivotEncoderPort);
        m_pivotPIDController = new PIDController(CoralGroundIntakeSubsystemConstants.kP, CoralGroundIntakeSubsystemConstants.kI, CoralGroundIntakeSubsystemConstants.kD);
        m_pivotFeedforward = new SimpleMotorFeedforward(CoralGroundIntakeSubsystemConstants.kS, CoralGroundIntakeSubsystemConstants.kV);

        m_intakeMotorFx.setNeutralMode(NeutralModeValue.Brake);
        m_intakeMotorFx.setSafetyEnabled(true);
        m_intakeMotorFx.setExpiration(0.1);

        m_pivotMotor.configure(
            Configs.CoralGroundIntakeSubsystem.pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        setDefaultCommand(
            runOnce(
                () -> {
                    m_intakeMotorFx.set(0);
                    m_pivotMotor.set(0);
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
    }

    @Override
    public void periodic() {
        m_intakeMotorFx.feed();
        SmartDashboard.putNumber("Pivot Encoder Position", getPivotEncoderPosition());
        SmartDashboard.putBoolean("Pivot Encoder Connected", isPivotEncoderConnected());

        SmartDashboard.putNumber("Pivot Motor Output", m_pivotMotor.get());
    }

    public double getPivotEncoderPosition() {
        return m_pivotEncoder.get();
    }

    public boolean isPivotEncoderConnected() {
        return m_pivotEncoder.isConnected();
    }

    public Command movePivotToPosition(double setpoint) {
        return parallel(
            run(
                () -> {
                    SmartDashboard.putNumber("Pivot Setpoint", setpoint);
                    double pidOutput = m_pivotPIDController.calculate(getPivotEncoderPosition(), setpoint);
                    //double feedforwardOutput = m_pivotFeedforward.calculate(setpoint);
                    //m_pivotMotor.set(pidOutput + feedforwardOutput);
                    pidOutput = Math.max(Math.min(pidOutput, 0.7), -0.4);
                    m_pivotMotor.set(pidOutput);
                    SmartDashboard.putNumber("Pivot PID Output", pidOutput);
                    //SmartDashboard.putNumber("Pivot Feedforward Output", feedforwardOutput);
                }),
            waitUntil(m_pivotPIDController::atSetpoint)
        ).withName("MovePivotToPosition");
    }

    public Command moveToStowPosition() {
        return movePivotToPosition(STOW_POSITION);
    }

    public Command moveToScorePosition() {
        return movePivotToPosition(SCORE_POSITION);
    }

    public Command moveToGroundPosition() {
        return movePivotToPosition(GROUND_POSITION);
    }

    public Command moveToCatchPosition() {
        return movePivotToPosition(CATCH_POSITION);
    }

    public Command movePivot(double d) {
        return run(() -> m_pivotMotor.set(d)).withName("MovePivot");
    }

    public Command moveIntake(double speed) {
        return new MoveIntakeCommand(speed);
    }

    // Inner class for operating the intake
    public class MoveIntakeCommand extends Command {
        private final double speed;

        public MoveIntakeCommand(double speed) {
            this.speed = speed;
            // Add requirements to ensure this command has exclusive access to the CoralGroundIntakeSubsystem
            addRequirements(CoralGroundIntakeSubsystem.this);
        }

        public void setIntakeMotor(double speed) {
            m_intakeMotorFx.set(speed);
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

}