package org.firstinspires.ftc.teamcode.Autonomous.kevin;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;



public class OdometryAutonMethods extends NewAutonMethods {

    public String[][] pos;

    OdometryAutonMethods()
    {
        pos = new String[9][18];
    }

    public void setEndCoord(int rowCoord, int colCoord)
    {

        for(int row = 0;row<pos.length;row++)
        {
            for(int col = 0; col< pos[row].length;col++)
            {
               if(row == rowCoord && col == colCoord )
               {
                   pos[row][col] = "goal";
               }
            }
        }
    }

    public void updateRobotCoord()
    {

    }


}
