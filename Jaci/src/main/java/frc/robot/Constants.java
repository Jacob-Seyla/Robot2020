/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static Trajectory drive8;
    
    public static void init(){
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 8.5, 10.0, 1000000.0);
				Trajectory.Config configFaster = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 10.0, 10.0, 1000000.0);
    
            Waypoint[] points8 = new Waypoint[] {
                // in feet
                new Waypoint(-25.0, 0, 0),
                new Waypoint(0, 0, 0)
            };
            drive8 = Pathfinder.generate(points8, config);
            File drive8file = new File("/home/lvuser/Trajectories/drive8.t");
            try {
                drive8file.createNewFile();
                Pathfinder.writeToFile(drive8file, drive8);
            } catch (IOException e) {
                e.printStackTrace();
            }

        try {
            drive8 = Pathfinder.readFromFile(new File("/home/lvuser/Trajectories/drive8.t"));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
