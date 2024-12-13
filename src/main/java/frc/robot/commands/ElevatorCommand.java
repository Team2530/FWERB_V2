package frc.robot.commands;

import frc.robot.subsystems.ElevatorSub;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorSub elevatorSub;
    private final double targetPosition;

    public ElevatorCommand(ElevatorSub elevatorSub, double targetPosition) {
        this.elevatorSub = elevatorSub;
        this.targetPosition = targetPosition;
        addRequirements(elevatorSub);
    }

    @Override
    public void initialize() {
        elevatorSub.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return elevatorSub.elevatorInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSub.disablePID();
    }
}
