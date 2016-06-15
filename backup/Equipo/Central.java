package Equipo;

import funciones.Acciones;
import funciones.Funciones;
import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;

public class Central extends Behaviour{

	Vec2 puntoZona1, puntoZona2;
	
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
		arg0.setDisplayString("Central");		
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		if(myRobotAPI.blocked() || myRobotAPI.teammateBlocking()){
			myRobotAPI.avoidCollisions();
			myRobotAPI.setSpeed(1.0);
			return RobotAPI.ROBOT_OK;
		}
		puntoZona1 = myRobotAPI.toEgocentricalCoordinates(new Vec2(-0.5, 0.7625));
		puntoZona2 = myRobotAPI.toEgocentricalCoordinates(new Vec2(0.5, -0.7625));
		if(myRobotAPI.closestToBall() || Funciones.estaEnZona(myRobotAPI.getBall(), puntoZona1, puntoZona2, myRobotAPI)){
			if(myRobotAPI.closestToBall() || Funciones.posesion(myRobotAPI)==0)
				if(myRobotAPI.closestToBall() && !myRobotAPI.opponentsHaveGoalkeeper()){
					myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
					myRobotAPI.kick();
				}else				
					if(myRobotAPI.toEgocentricalCoordinates(new Vec2(0,0)).r<=0.2)
						myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
					else
						myRobotAPI.passBall(myRobotAPI.getClosestMate());
			else
				myRobotAPI.blockClosest();
			myRobotAPI.setSpeed(1.0);
			return RobotAPI.ROBOT_OK;
		}		
		//Acciones.patrullaZona(myRobotAPI, puntoZona1, puntoZona2);
		myRobotAPI.setSteerHeading(myRobotAPI.toEgocentricalCoordinates(new Vec2(0,0)).t);
		myRobotAPI.setSpeed(1.0);
		return RobotAPI.ROBOT_OK;
	}

}
