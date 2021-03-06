/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Chain extends SequentialCommandGroup {
  /**
   * Creates a new Chain.
   * 
   * @throws IOException
   */
  public Chain() throws IOException {
    // Add your commands in the super() call, e.g.
    // super(new Forward(), new PathweaverPath());
    // super(new PathweaverPath(), new Forward());
    addCommands(
      new Forward().withTimeout(1.15),
      new PathweaverPath()
    );
  }
}
