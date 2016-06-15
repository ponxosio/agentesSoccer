package Equipo;

import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;

public class BlockPortero extends Behaviour {

	@Override
	public void configure() {
		// TODO Auto-generated method stub

	}

	@Override
	public void end() {
		// TODO Auto-generated method stub

	}

	@Override
	public void onInit(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		myRobotAPI.blockGoalKeeper();
		myRobotAPI.setSpeed(1);
		return 0;
	}

}
