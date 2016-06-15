package Equipo;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;

public class GoToBall extends Behaviour {	
	
	@Override
	public void configure() {
		// TODO Auto-generated method stub

	}

	@Override
	public void end() {
		// TODO Auto-generated method stub

	}

	@Override
	public void onInit(RobotAPI r) {
		// TODO Auto-generated method stub
		r.setDisplayString("gotoballbehaviour");
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
		if(myRobotAPI.canKick()){
			myRobotAPI.kick();
			
		}
		return RobotAPI.ROBOT_OK;
	}

}
