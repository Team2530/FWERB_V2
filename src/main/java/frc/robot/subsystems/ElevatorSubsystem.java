package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorSubsystem extends ProfiledPIDSubsystem {
    // NOTE: Elevator motor one has both the encoder used for positioning, and the
    // limit switch used for zeroing
    private final CANSparkFlex elevatorMotorOne = new CANSparkFlex(Constants.Elevator.elevatorOnePort,
            MotorType.kBrushless);
    private final CANSparkFlex elevatorMotorTwo = new CANSparkFlex(Constants.Elevator.elevatorTwoPort,
            MotorType.kBrushless);
    private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
            Constants.Elevator.Feedforward.Ks,
            Constants.Elevator.Feedforward.Kg,
            Constants.Elevator.Feedforward.Kv,
            Constants.Elevator.Feedforward.Ka

    );

    /*
     * DCMotor gearbox,
     * double gearing,
     * double carriageMassKg,
     * double drumRadiusMeters,
     * double minHeightMeters,
     * double maxHeightMeters,
     * boolean simulateGravity,
     * double startingHeightMeters,
     * Matrix<N1, N1> measurementStdDevs
     */
    private boolean isZeroed = false;
    private final RelativeEncoder elevatorEncoder = elevatorMotorOne.getEncoder();
    private final SparkLimitSwitch bottomLimit;

    private final ElevatorSim simulation = new ElevatorSim(
            Constants.Elevator.PhysicalParameters.simMotor,
            Constants.Elevator.PhysicalParameters.gearReduction,
            Constants.Elevator.PhysicalParameters.carriageMassKg,
            Constants.Elevator.PhysicalParameters.driveRadiusMeters,
            0.0,
            Constants.Elevator.PhysicalParameters.elevatorHeightMeters,
            true, 0.0,
            VecBuilder.fill(0.01));

    public ElevatorSubsystem() {
        super(
                new ProfiledPIDController(
                        Constants.Elevator.PID.kP,
                        Constants.Elevator.PID.kI,
                        Constants.Elevator.PID.kD,
                        new TrapezoidProfile.Constraints(
                                Constants.Elevator.PID.MAX_VELOCITY,
                                Constants.Elevator.PID.MAX_ACCELERATION)),
                0.0);

        // TODO: Check that this is an acceptable tolerance!
        this.getController().setTolerance(Units.inchesToMeters(0.5));

        isZeroed = false;

        elevatorMotorOne.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        elevatorMotorTwo.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        elevatorEncoder.setPositionConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);
        elevatorEncoder.setVelocityConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);
        // elevatorEncoder.setInverted(Constants.Elevator.elevatorEncoderInverted);

        bottomLimit = elevatorMotorOne.getReverseLimitSwitch(Constants.Elevator.bottomLimitMode);

        this.enable();
        // todo check if offset encoders
        // todo check if motors inverted
        // todo set elevator tolerance
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // TODO: Elevator feedforward
        // double feedforward = m_feedforward.calculate(setpoint.position,
        // setpoint.velocity);
        double ff = feedForward.calculate(setpoint.velocity, 0.0);

        if (isZeroed || Robot.isSimulation()) {
            elevatorMotorOne.setVoltage(output + ff);
            elevatorMotorTwo.setVoltage(output + ff);
            SmartDashboard.putNumber("Elevator Total Output", output + ff);
            SmartDashboard.putNumber("Elevator PID Output", output);
            SmartDashboard.putNumber("Elevator FF Output", ff);
            SmartDashboard.putNumber("Current Elevator Target (profiled)", setpoint.position);
            if (Robot.isSimulation()) {
                simulation.setInputVoltage(MathUtil.clamp(output + ff, -12.0, 12.0));
            }
        } else {
            // Move down a little bit to zero
            elevatorMotorOne.set(-0.05);
            elevatorMotorTwo.set(-0.05);
        }
    }

    @Override
    public double getMeasurement() {
        return Robot.isSimulation() ? simulation.getPositionMeters() : elevatorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        if (bottomLimit.isPressed() && (!isZeroed)) {
            elevatorEncoder.setPosition(0.0);
            setGoal(0.0);
            isZeroed = true;
        }

        SmartDashboard.putNumber("Current Elevator Position", getMeasurement());
        SmartDashboard.putNumber("Goal Elevator Position", this.getController().getGoal().position);
        SmartDashboard.putBoolean("Elevator Zeroed", isZeroed);
        SmartDashboard.putBoolean("Elevator Bottom Limit", bottomLimit.isPressed());
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();

        // Next, we update it. The standard loop time is 20ms.

        simulation.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage

        // SimBattery estimates loaded battery voltages

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simulation.getCurrentDrawAmps()));
    }

    public boolean isInPosition() {
        return this.getController().atGoal();
    }
}