package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Lower arm from fully up position to correct height for high scoring,
 * extend the arm to back scoring position.
 */
public class ScoreHigh extends CommandBase 
{
    private final Winch             winch;
    private final Arm               arm;
    private final Claw              claw;
    private ParallelCommandGroup    pCommands; 

    public ScoreHigh(Winch winch, Arm arm, Claw claw)
    {
        Util.consoleLog();

        this.winch = winch;
        this.arm = arm;
        this.claw = claw;

        addRequirements(winch, arm, claw);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

		// Commands will be run in parallel.

		pCommands = new ParallelCommandGroup();

		// First action is to lower the arm.

		Command command = new LowerArm(winch, -40);

		pCommands.addCommands(command);

        // Now hold winch position.

        command = new InstantCommand(winch::toggleHoldPosition);

		pCommands.addCommands(command);

        // Next action is to extend arms.

        command = new ExtendArm(arm, 100);

		pCommands.addCommands(command);

        // Run the commands, only if winch fully up.

        if (winch.getUpperSwitch()) pCommands.schedule();

        SmartDashboard.putBoolean("ScoreHigh", true);
    }

    @Override
    public boolean isFinished()
    {
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		return !pCommands.isScheduled();
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        if (interrupted) pCommands.cancel();

        SmartDashboard.putBoolean("ScoreHigh", false);
    }
}

