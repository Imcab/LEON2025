package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Components.AlgaeWrist;

public class AlgaeCommands {

    public static Command defaultAlgae(AlgaeWrist algae, double position, double wheelSpeed){
        return Commands.run(()->{

            algae.setPosition(position);
            algae.runWheels(wheelSpeed);

        },
        algae)

        .finallyDo(algae::stop);
    }

    public static Command noStopAlgae(AlgaeWrist algae, double position, double wheelSpeed){
        return Commands.run(()->{

            algae.setPosition(position);
            algae.runWheels(wheelSpeed);

        },
        algae);
    }

    public static Command autoAlgae(AlgaeWrist algae, double position, double wheelSpeed){
        return Commands.run(()->{

            algae.setPosition(position);
            algae.runWheels(wheelSpeed);

        },
        algae)

        .finallyDo(()->algae.runWheels(0));
    }

    public static Command shootAlgae(AlgaeWrist algae, double wheelSpeed){
        return Commands.run(()->{

            algae.runWheels(wheelSpeed);

        },
        algae)

        .finallyDo(algae::stop);
    }

    


}
