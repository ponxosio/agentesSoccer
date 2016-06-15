package Equipo;

import java.util.Queue;

import funciones.Acciones;
import funciones.Funciones;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

public class Patrullero extends Behaviour {
	Vec2 punto;
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
		punto = arg0.getPosition();
		arg0.setDisplayString("Patrullero");
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		Queue<Message> mensajes = getPendingMessages();
		if(mensajes.isEmpty()){
			if(myRobotAPI.blocked() || myRobotAPI.teammateBlocking()){
				myRobotAPI.avoidCollisions();
				myRobotAPI.setSpeed(1.0);		
				return RobotAPI.ROBOT_OK;
			}else{
				Acciones.patrullaOctante(myRobotAPI, punto);
				return RobotAPI.ROBOT_OK;
			}
		}else{
			return RobotAPI.ROBOT_OK;
		}
		
	}

}
