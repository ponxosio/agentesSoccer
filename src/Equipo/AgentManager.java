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
        return _behaviours[i];
    }

    @Override
    public Behaviour[] createBehaviours() {
        Behaviour[] vuelta = {new AgentPlayer(this, 0), new AgentPlayer(this, 1), new AgentPlayer(this, 2), new AgentPlayer(this, 3), new AgentPlayer(this, 4)};
        return vuelta;
    }
}
