package frc.robot.BulukLib.PathPlanner;
import java.util.Set;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FinishWhenPathCommand extends Command{
    
    private final Command other;
    private boolean lastState;
    private final PathPlannerAuto CurrentAuto;
    private final String PathName;

    public FinishWhenPathCommand(PathPlannerAuto CurrentAuto, String PathName, Command other){
        this.other = other;
        this.CurrentAuto = CurrentAuto;
        this.PathName = PathName;
        
        Set<Subsystem> requirements = other.getRequirements();
        addRequirements(requirements.toArray(new Subsystem[0]));
    }

    @Override
    public void initialize(){
        other.schedule();
        lastState = CurrentAuto.activePath(PathName).getAsBoolean();
    }

    @Override
    public void end(boolean interrupted){
        other.cancel();
    }

    @Override
    public boolean isFinished(){
        boolean currentState = CurrentAuto.activePath(PathName).getAsBoolean();
        boolean isFallingEdge = lastState && !currentState;
        lastState = currentState;
        return isFallingEdge;
    }

}
