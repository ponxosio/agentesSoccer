package Equipo;

import java.util.Queue;

import EDU.gatech.cc.is.util.Vec2;

import mensajes.BlockMessage;

import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

public class Blocker extends Behaviour {

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
		arg0.setDisplayString("Blocker");
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		Queue<Message> mensajes = getPendingMessages();
		if(mensajes.isEmpty()){
			myRobotAPI.setDisplayString("Bl-quien quiero");
			if(myRobotAPI.blocked()||myRobotAPI.teammateBlocking()){
				myRobotAPI.avoidCollisions();
				myRobotAPI.setSpeed(1.0);		
				return RobotAPI.ROBOT_OK;
			}
			else{
				myRobotAPI.blockGoalKeeper();
				myRobotAPI.setSpeed(1.0);		
				return RobotAPI.ROBOT_OK;
			}
		}else{
			myRobotAPI.setDisplayString("Bl-mensaje");
			for(Message m: mensajes){
				if(m.getClass() == BlockMessage.class){
					Vec2 op = myRobotAPI.toEgocentricalCoordinates(((BlockMessage)m).getPunto());
					if(op.r<=0.2)
						myRobotAPI.blockClosest();
					else
						myRobotAPI.setSteerHeading(op.t);
					myRobotAPI.setSpeed(1.0);
					
				}
			}
			return RobotAPI.ROBOT_OK;
		}
	}

}
