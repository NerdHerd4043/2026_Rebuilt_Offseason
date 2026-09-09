package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;

@Logged
public class Intake extends SubsystemBase {
    private SparkFlex intakingMotor = new SparkFlex(IntakeConstants.intakingMotorID, MotorType.kBrushless);
    private SparkMax articulatingMotor = new SparkMax(IntakeConstants.articulatingMotorID, MotorType.kBrushless);

    private CANcoder encoder = new CANcoder(28);
    private ArmFeedforward feedforward = new ArmFeedforward(
            IntakeConstants.kS,
            IntakeConstants.kG,
            IntakeConstants.kV);

    private ProfiledPIDController pidController = new ProfiledPIDController(IntakeConstants.P, IntakeConstants.I,
            IntakeConstants.D, new TrapezoidProfile.Constraints(2, 3));

    private boolean resting = true;

    public Intake() {
        final SparkFlexConfig intakingMotorConfig = new SparkFlexConfig();
        final SparkMaxConfig articulatingMotorConfig = new SparkMaxConfig();

        intakingMotorConfig.idleMode(IdleMode.kBrake);

        articulatingMotorConfig.idleMode(IdleMode.kCoast);
        articulatingMotorConfig.smartCurrentLimit(IntakeConstants.articulatingMotorCurrent);

        intakingMotor.configure(intakingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        articulatingMotor.configure(articulatingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pidController.disableContinuousInput();
    }

    public Command runIntake() {
        return this.run(() -> {
            intakingMotor.set(IntakeConstants.intakeSpeed);
        }).finallyDo(() -> {
            intakingMotor.stopMotor();
        });
    }

    public Command intakeToStartAngle() {
        return this.runOnce(() -> {
            pidController.setGoal(IntakeConstants.startingAngle);
        });
    }

    public Command intakeToIntakeAngle() {
        return this.runOnce(() -> {
            resting = false;
            pidController.setGoal(IntakeConstants.intakeAngle);
        });
    }

    public Command helpFeedBalls() {
        resting = false;

        Command moveIntakeUp = runOnce(() -> {
            pidController.setGoal(45);
        }).withTimeout(2);

        Command moveIntakeDown = runOnce(() -> {
            pidController.setGoal(0);
        }).withTimeout(2);

        Command helpFeedBallsCommand = Commands.sequence(moveIntakeUp, moveIntakeDown);

        return helpFeedBallsCommand;
    }

    public Command intakeUp() {
        return this.run(() -> {
            articulatingMotor.set(0.2);
        }).finallyDo(() -> {
            articulatingMotor.stopMotor();
        });
    }

    @Override
    public void periodic() {

        if (!resting && !pidController.atSetpoint()) {
            double encoderPos = encoder.getAbsolutePosition().getValueAsDouble();
            double PIDOutput = -pidController.calculate(encoderPos);

            double PIDSetPointVelocity = pidController.getSetpoint().velocity;
            double PIDSetPointPos = pidController.getSetpoint().position;
            double FFOutput = -feedforward.calculate(PIDSetPointPos, PIDSetPointVelocity);

            double Voltage = PIDOutput + FFOutput;

            articulatingMotor.setVoltage(Voltage);
        }

        // if (!resting && !pidController.atSetpoint()) {
        // articulatingMotor.setVoltage(pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble())
        // + feedforward.calculate(pidController.getSetpoint().position,
        // pidController.getSetpoint().velocity));
        // }

        SmartDashboard.putNumber("Intake Duty Cycle In %", articulatingMotor.getAppliedOutput());
        SmartDashboard.putNumber("Encoder ABS Pos", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("PID Set Point", pidController.getSetpoint().position);
    }

    public void disabledInit() {
        resting = true;
    }
}
