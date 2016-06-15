package Equipo;

import java.util.Queue;
import java.util.Random;

import funciones.Acciones;
import funciones.Funciones;

import EDU.gatech.cc.is.util.Vec2;


import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

public class GoalKeeper extends Behaviour {
	//pongo estos valores para que sea un punto fuera del campo
		private Vec2 pant = new Vec2(5, 5); 
	
	@Override
	public void configure() {
		// TODO Auto-generated method stub

	}

	@Override
	public void end() {
		// TODO Auto-generated method stub

	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		Queue<Message> mensajes = getPendingMessages();
		if(mensajes.isEmpty()){
			Vec2 posbalon = myRobotAPI.getBall();
			//si el portero esta bloqueado, lo evita
			if (myRobotAPI.blocked() || myRobotAPI.teammateBlocking()){
				myRobotAPI.avoidCollisions(); 
				myRobotAPI.setSpeed(1.0);
				pant=posbalon;
				return RobotAPI.ROBOT_OK;
			}
			//si el balon está cerca del portero lo despeja o pasa al jugador más cercano
			if(myRobotAPI.getOurGoal().r<=0.3 && posbalon.r<0.3 || myRobotAPI.getOurGoal().distance(posbalon)<0.3 || (pant.x!=5 && Funciones.trayectoriaDeGol(pant, posbalon, myRobotAPI))){
				if(((Double)myRobotAPI.passHeuristic(myRobotAPI).value)<0.5){
					Vec2 p = (Vec2) myRobotAPI.getOpponentsGoal().clone();
					p.sett(p.t+Math.random()*Vec2.PI*((Math.random()>=0.5)?1:-1)/8);
					myRobotAPI.setBehindBall(p);
					myRobotAPI.setSpeed(1.0);
					myRobotAPI.kick();
				}else{
					myRobotAPI.setSpeed(1.0);
					myRobotAPI.passBall(myRobotAPI.getClosestMate());				
				}
			}else{
				//por defecto patrulla el area
				//Acciones.patrullaPunto(myRobotAPI, myRobotAPI.getOurGoal());
				Acciones.paraleloAlBalonEjeY(myRobotAPI);
			}
			pant=posbalon;
			return RobotAPI.ROBOT_OK;
		}else{
			return RobotAPI.ROBOT_OK;
		}
	}
	
	@Override
	public void onInit(RobotAPI r) {
	r.setDisplayString("Kepper");
	}
	
}
