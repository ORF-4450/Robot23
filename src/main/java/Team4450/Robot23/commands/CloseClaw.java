package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the Claw to a target position.
 */
public class CloseClaw extends CommandBase 
{
    private final Claw      claw;
    private double          targetPostion = 100;    // Revolutions of motor.
    private SynchronousPID  controller = new SynchronousPID(.01, 0, 0);
    private final double    tolerance = .5, maxPower = .30;
    private double          lastTimeCalled;

    /**
     * Move claw to target position (closing it).
     * @param winch Winch subsystem.
     * @param targetPosition Target position in winch motor revolutions.
     */
    public CloseClaw(Claw claw, double targetPosition)
    {
        Util.consoleLog();

        this.claw = claw;

        this.targetPostion = targetPosition;

        addRequirements(claw);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        controller.setSetpoint(targetPostion);

        controller.setOutputRange(-maxPower, maxPower);

        SmartDashboard.putBoolean("CloseClaw", true);

        lastTimeCalled = Util.timeStamp();
    }

    @Override
    public void execute()
    {
        double time = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        double power = controller.calculate(claw.getPosition(), time);

        claw.setPower(power);
    }

    @Override
    public boolean isFinished()
    {
        return controller.onTarget(tolerance);
    }

    @Override
    public void end(boolean interrupted) 
    {
        claw.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("CloseClaw", false);
    }
}
