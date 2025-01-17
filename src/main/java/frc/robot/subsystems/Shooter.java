package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase {
  // The motor on the shooter wheel .
  private final TalonFX motor = new TalonFX(ShooterConstants.kShooterMotorPort);
  private final VoltageOut voltageOut = new VoltageOut(0);

  // The shooter wheel encoder

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (voltage) -> motor.setControl(voltageOut.withOutput(voltage)),
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            motor.getMotorVoltage().getValue()))
                    .angularPosition(m_angle.mut_replace(motor.getPosition().getValue()))
                    .angularVelocity(
                        m_velocity.mut_replace(motor.getVelocity().getValue()));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  // PID controller to run the shooter wheel in closed-loop, set the constants equal to those
  // calculated by SysId
  private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0, 0);
  // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
  // those calculated by SysId
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts,
          ShooterConstants.kVVoltSecondsPerRotation,
          ShooterConstants.kAVoltSecondsSquaredPerRotation);

  /** Creates a new Shooter subsystem. */
  public Shooter() {
    // Sets the distance per pulse for the encoders
  }

  /**
   * Returns a command that runs the shooter at a specifc velocity.
   *
   * @param shooterSpeed The commanded shooter wheel speed in rotations per second
   */


  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
