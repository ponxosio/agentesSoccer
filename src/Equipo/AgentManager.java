package Equipo;


import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;
import teams.ucmTeam.TeamManager;

/**
 * Created by angel on 21/01/2016.
 */
public class AgentManager extends TeamManager {

    @Override
    public int onConfigure() {
        return RobotAPI.ROBOT_OK;
    }

    @Override
    protected void onTakeStep() {

    }

    @Override
    public Behaviour getDefaultBehaviour(int i) {
        return _behaviours[Math.min(i,1)];
    }

    @Override
    public Behaviour[] createBehaviours() {
        Behaviour[] vuelta = new Behaviour[2];
        vuelta[0] = new Passer();
        return vuelta;
    }
}
