package frc.robot.Subsystems.Components.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Components.Elevator.ElevatorControl.ElevatorState;

public class Elevator extends SubsystemBase{

  public enum ElevatorLevels{
      kGround, kL2ToL3, kL3toL4;
  }

  private ElevatorBuilder elevator;
  private ElevatorState elevatorInputs;
  private DigitalInput limit;
  private boolean stopMotion = false;
  public boolean atHome = false;
  public boolean atGoal = false;
  private double previousVelocity = 0.0;
  private double kS = 0;
  private double kA = 0;
  private double kG = 0;
  private boolean shouldEStop = false;
  private boolean isEStopped = false;
  private static final double tolerance = 0.2; //meters
  private final Debouncer toleranceDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);
  private ElevatorLevels currentLevel = ElevatorLevels.kGround;

  public Elevator(ElevatorBuilder elevator) {

    limit = new DigitalInput(0);
    this.elevator = elevator;

    elevator.configureElevator();
  }

  public void setGoal(double position){
    elevator.getController().setGoal(position);
  }

  @Override
  public void periodic() {

    elevator.updateState(elevatorInputs);

    if (ElevatorConstants.ground_l2.inRange(elevatorInputs.meters)) {
      currentLevel = ElevatorLevels.kGround;
    } else if (ElevatorConstants.l2_l3.inRange(elevatorInputs.meters)) {
      currentLevel = ElevatorLevels.kL2ToL3;
    } else if (ElevatorConstants.l3_l4.inRange(elevatorInputs.meters)) {
      currentLevel = ElevatorLevels.kL3toL4;
    }else{
      currentLevel = ElevatorLevels.kL3toL4; //por si falla
    }

    //Actualiza diferentes valores de feedforward dependiendo del nivel del elevador
    switch (currentLevel) {
      case kGround:
        kS = ElevatorConstants.COMPstaticFriction[0];
        kA = ElevatorConstants.COMPaccFactor[0];
        kG = ElevatorConstants.COMPgravityFactor[0];
        break;
      case kL2ToL3:
        kS = ElevatorConstants.COMPstaticFriction[1];
        kA = ElevatorConstants.COMPaccFactor[1];
        kG = ElevatorConstants.COMPgravityFactor[1];
        break;
      case kL3toL4:
        kS = ElevatorConstants.COMPstaticFriction[2];
        kA = ElevatorConstants.COMPaccFactor[2];
        kG = ElevatorConstants.COMPgravityFactor[2];
        break;
      default:

        kS = 0;
        kA = 0;
        kG = 0;

        break;
    }

    SmartDashboard.putBoolean("MTY", isMTY());
    SmartDashboard.putNumber("ELEVATOR VELOCITY", getVelocity());
    SmartDashboard.putBoolean("SWITCH PRESSED", pressed());
    SmartDashboard.putNumber("ELEVATOR HEIGHT", getMeters());
  
    if (pressed()) {
      atHome = true;
      resetEncoders();
    }

    atHome = retracted();

    final boolean shouldRunProfile = 
    !stopMotion && atHome && DriverStation.isEnabled() && !isEStopped;

    boolean outOfTolerance = Math.abs(elevatorInputs.meters - elevatorInputs.positionGoal) > tolerance;
        shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);

        if (shouldEStop) {
            isEStopped = true;
        }

      if (shouldRunProfile) {
        double positionGoal = elevatorInputs.positionGoal;
        double currentPosition = elevatorInputs.meters;
        double currentVel = elevatorInputs.velocityMetersPerSec;
        double acc = (currentVel - previousVelocity) / 0.02;

        boolean isPositionValid = ElevatorConstants.physicalCapacity.inRange(positionGoal);

        if (isEStopped) {
          elevator.stop();
        }
    
        if (isPositionValid) {

          double pidOutput = elevator.getController().run(currentPosition);
          
          if (!isMTY()) {
            double voltageOutput = elevator.getController().run(currentPosition) + Math.signum(currentVel) * kS  +  kG + kA * acc ;
            elevator.setVoltage(voltageOutput);
          } else {
            elevator.setSpeed(pidOutput);
            
          }
        }
        else{
          positionGoal = MathUtil.clamp(positionGoal, ElevatorConstants.physicalCapacity.minValue(), ElevatorConstants.physicalCapacity.maxValue());
        }
        
        atGoal = 
          Math.abs(currentPosition - positionGoal)
           <= elevator.currentTolerance().getPositionTolerance() && Math.abs(currentVel)
            <= elevator.currentTolerance().getVelocityTolerance();

        previousVelocity = currentVel;

      }
      
      else{
        elevator.getController().run(elevatorInputs.meters, 0.0); //reset setpoint
      }

  }

  public void resetEncoders() {
    elevator.setEncoderPosition(0);
  }

  public boolean pressed(){
    return !limit.get();
  }

  public double getVelocity() {
    return elevatorInputs.velocityMetersPerSec;
  }

  public double getMeters() {
    return elevatorInputs.meters;
  }

  public boolean isMTY(){
    return elevator.isMTY();
  }

  public boolean retracted(){
    return getMeters() <= ElevatorControl.kHome + 0.02 || pressed();
  }

  public void runSpeed(double speed) {
    elevator.setSpeed(speed);
  }

  public void stop() {
    elevator.stop();
  }

  public Command goToHome(){

    return Commands.startRun(()-> {
      atHome = false;
      stopMotion = true;
    }, ()->{
      elevator.setSpeed(-0.2); // bajar
      atHome = pressed();
    }).until(()-> atHome).andThen(()->{
      resetEncoders();
      atHome = true;
    }).finallyDo(()-> {
      elevator.stop();
      elevator.getController().run(elevatorInputs.meters, 0.0);
    });
  }

}
