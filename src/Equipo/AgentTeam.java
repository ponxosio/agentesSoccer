package Equipo;

import teams.ucmTeam.TeamManager;
import teams.ucmTeam.UCMPlayer;

/**
 * Created by angel on 21/01/2016.
 */
public class AgentTeam extends UCMPlayer {

    @Override
    protected TeamManager getTeamManager() {
        return new AgentManager();
    }
}
