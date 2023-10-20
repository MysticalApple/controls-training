// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ExampleSubsystem extends SubsystemBase {

  private final CANSparkMax flywheel;
  private final CANSparkMax flywheel2;
  private final Talon belts;

  private final AnalogPotentiometer lower;
  private final AnalogPotentiometer center;
  private final AnalogPotentiometer upper;

  private final double lowerSensorCutoff = 0.2;
  private final double centerSensorCutoff = 0.2;
  private final double upperSensorCutoff = 0.2;

  boolean started = false;
  Timer timer;

  boolean[] ballStorage;
  

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    flywheel = new CANSparkMax(3, MotorType.kBrushless);
    flywheel2 = new CANSparkMax(16, MotorType.kBrushless);
    flywheel2.follow(flywheel);
    belts = new Talon(8);

    lower = new AnalogPotentiometer(0);
    center = new AnalogPotentiometer(1);
    upper = new AnalogPotentiometer(2);


    timer = new Timer();
    
    ballStorage = new boolean[]{false, false, false};
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  
  


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!started) {
      started = true;
      timer.start();
    }

    if (started && timer.hasElapsed(1)) {
      System.out.println("\n\n");
      System.out.println("Lower: " + lower.get());
      System.out.println("Center: " + center.get());
      System.out.println("Upper: " + upper.get());
  
      // Update Ball Storage
      ballStorage[0] = upper.get() > upperSensorCutoff;
      ballStorage[1] = center.get() > centerSensorCutoff;
      ballStorage[2] = lower.get() > lowerSensorCutoff;

      timer.restart();
    }  

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
