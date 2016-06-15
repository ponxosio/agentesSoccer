package Equipo;

import teams.ucmTeam.TeamManager;
import teams.ucmTeam.UCMPlayer;

public class ElPayoVallecano extends UCMPlayer {

	@Override
	protected TeamManager getTeamManager() {
		// TODO Auto-generated method stub
		return new Manager();
	}

}
