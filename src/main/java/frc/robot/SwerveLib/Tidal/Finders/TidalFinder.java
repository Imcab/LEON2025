package frc.robot.SwerveLib.Tidal.Finders;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;
import frc.robot.SwerveLib.Tidal.TidalUtils.Coordinate;
import frc.robot.SwerveLib.Tidal.TidalUtils.Grid;
import frc.robot.SwerveLib.Tidal.TidalUtils.TidalUtil;

//Pathfinder generator
public class TidalFinder extends SubsystemBase{
    
    private Coordinate goal;
    private swerve drive;
    private PathConstraints gains;
    private double acc;
  
    public TidalFinder(){
        goal = new Coordinate(0, 0);
           
    }

    public void linkVehicle(swerve drive){
        if(this.drive != null){
            throw new IllegalStateException("Vehicle already linked!");
        }

        this.drive = drive;
    }

    public void setNavigationInaccuracy(double percentage){
        this.acc = percentage;
    }

    public BooleanSupplier isBlue(){
        return ()-> TidalUtil.isBlue();
    }

    public void setConstraints(PathConstraints gains){
        this.gains = gains;
    }
    
    private Command toSingleCoordinate(Coordinate goal, PathConstraints gainsCustom){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gainsCustom)
        .beforeStarting(()-> {
            drive.offLeds(true);
            drive.blinkin.setPattern(PatternType.FireLarge);
            this.goal = goal;
        }).onlyWhile(()->!atGoal()).finallyDo(()->{
            drive.offLeds(false);
            stopVehicle();});
        finder.addRequirements(drive);
        return finder;
    }

    private Command toSingleCoordinate(Coordinate goal){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gains)
        .beforeStarting(()-> {
            drive.offLeds(true);
            drive.blinkin.setPattern(PatternType.FireLarge);
            this.goal = goal;
        }).onlyWhile(()->!atGoal()).finallyDo(()->{
            drive.offLeds(false);
            stopVehicle();});
        finder.addRequirements(drive);
        return finder;
    }

    public Command navigateToPathPlannerPath(String pathPlannerPath){

        try{
            PathPlannerPath objective = PathPlannerPath.fromPathFile(pathPlannerPath);
            return AutoBuilder.pathfindThenFollowPath(objective, gains);
        }catch (Exception e){
            DriverStation.reportError(pathPlannerPath + ": Not Found!", false);
            return Commands.none();
        }
        
    }

    public ConditionalCommand navigateToCoordinate(Coordinate coordinate, PathConstraints gains){

        return new ConditionalCommand(toSingleCoordinate(coordinate, gains), toSingleCoordinate(TidalUtil.coordinateFlip(coordinate), gains), isBlue());

    }

    public ConditionalCommand navigateToCoordinate(Coordinate coordinate){

        return new ConditionalCommand(toSingleCoordinate(coordinate), toSingleCoordinate(TidalUtil.coordinateFlip(coordinate)), isBlue());

    }

    public Command setVehicleCoordinate(Coordinate newCoordinate){
        return Commands.runOnce(()-> drive.setPose(newCoordinate.toPose2d()), drive);
    }

    public boolean atGoal(){
        return new Grid(goal, acc).atGrid(TidalUtil.pose2dToCoordinate(drive.getPose()));
    }

    public Coordinate getGoal(){
        return goal;
    }

    public double getInaccuracy(){
        return acc;
    }

    public Coordinate lastCoordinate(){
        return TidalUtil.pose2dToCoordinate(drive.getPose());
    }

    public void stopVehicle(){
        drive.stop();
    }

    @Override
    public void periodic(){
        
    }


}
