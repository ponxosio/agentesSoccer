package Equipo;

import java.util.Queue;

import funciones.Acciones;
import funciones.Funciones;

import EDU.gatech.cc.is.util.Vec2;
import t101116.acciones.IrABalon;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;
import teams.ucmTeam.RobotAPI.Pair;

public class Delantero extends Behaviour{

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
		r.setDisplayString("Delantero");
		
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		Queue<Message> mensajes = getPendingMessages();
		if(mensajes.isEmpty()){
			Vec2 posBall = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
			Vec2 posicion = myRobotAPI.getPosition();
			if(myRobotAPI.closestToBall()){
				if(myRobotAPI.canKick() && myRobotAPI.alignedToBallandGoal()){
					myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
					myRobotAPI.kick();
					return RobotAPI.ROBOT_OK;
				}
				if(Funciones.oponenteEnFrente(myRobotAPI) || myRobotAPI.blocked() || myRobotAPI.teammateBlocking()){
					Acciones.esquivaOPasa(myRobotAPI);
					return RobotAPI.ROBOT_OK;
				}					
				myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
			}else{
					
				int posesion = Funciones.posesion(myRobotAPI);
				double campoContrario =myRobotAPI.getFieldSide()*(-1);
				//posesion equipo contrario
				if(posesion<0){				
					if((posBall.x*campoContrario>=0))
						myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
					else
						//comprobamos si estamos en el campo contrario
						if(posicion.x*campoContrario>0)
							Acciones.patrullaOctante(myRobotAPI, posicion);
						else
							//vamos a un punto aleatorio del campo contrario, fuera del area y centrado.
							Acciones.irAPunto(myRobotAPI,myRobotAPI.toEgocentricalCoordinates(new Vec2(Math.random()*1.145*campoContrario, Math.random()*0.5*((Math.random()>=0.5)?1:-1))));
					myRobotAPI.setSpeed(1.0);
					return RobotAPI.ROBOT_OK;
				}
				//posesion de un compañero
				if(posesion>0){
					//comprobamos si estamos en el campo contrario
					if(posicion.x*campoContrario>0)
						if(myRobotAPI.getClosestOpponent().distance(posicion)<0.2)					
							myRobotAPI.blockClosest();
						else
							Acciones.patrullaOctante(myRobotAPI, posicion);
					else
						//vamos a un punto aleatorio del campo contrario, fuera del area y centrado.
						Acciones.irAPunto(myRobotAPI, new Vec2(Math.random()*1.145*campoContrario, Math.random()*0.5*((Math.random()>=0.5)?1:-1)));
					myRobotAPI.setSpeed(1.0);
					return RobotAPI.ROBOT_OK;
				}
				myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
			}
			return RobotAPI.ROBOT_OK;
		}else{
			return RobotAPI.ROBOT_OK;
		}
	}

}
