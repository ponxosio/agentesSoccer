package Equipo;

import java.util.Queue;

import funciones.Acciones;
import funciones.Funciones;


import EDU.gatech.cc.is.util.Vec2;

import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

public class KeeperSinDespejar extends Behaviour {
	private Vec2 posa= new Vec2(5,5);
	private double area=0.1;
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
		arg0.setDisplayString("porteroND");
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		Queue<Message> mensajes = getPendingMessages();
		Vec2 posbalon = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
		Vec2 pos = myRobotAPI.getPosition();
		double bx=posbalon.x*myRobotAPI.getFieldSide();
		double miy=pos.y;
		double mix=pos.x*myRobotAPI.getFieldSide();
		double ang=pos.t;
		if(mensajes.isEmpty()){
			//si el portero esta bloqueado, lo evita
			/*sino se me cuelan lo tiene que hacer el manager*/
			if(mix<1.25){//tiene que volver a la porteria
				//avanza a la porteria
				if (myRobotAPI.blocked()){
					myRobotAPI.setDisplayString("P-colision");
					myRobotAPI.avoidCollisions(); 
					myRobotAPI.setSpeed(1.0);
					return RobotAPI.ROBOT_OK;
				}
					myRobotAPI.setDisplayString("P-a porteria");
					myRobotAPI.setSteerHeading(myRobotAPI.getOurGoal().t);
					myRobotAPI.setSpeed(1.0);
					return RobotAPI.ROBOT_OK;
			}
			else{//esta en la porteria para	
				if(bx>0.5){//esta en mi campo
						if(posbalon.y > miy){ 
							if(miy<0.25){//subo si no estoy muy arriba
								myRobotAPI.setSteerHeading(Math.PI/2);
								myRobotAPI.setDisplayString("P-subo");
								myRobotAPI.setSpeed(1.0);
								return RobotAPI.ROBOT_OK;
							}else{
								myRobotAPI.setDisplayString("P-espero");
								myRobotAPI.setSteerHeading(myRobotAPI.getBall().t);
								myRobotAPI.setSpeed(0.0);
								return RobotAPI.ROBOT_OK;
							}
						}else{
							if(posbalon.y < miy){
								if(miy>-0.25){//bajo si no estoy muy abajo
									myRobotAPI.setSteerHeading(-Math.PI/2);
									myRobotAPI.setDisplayString("P-bajo");
									myRobotAPI.setSpeed(1.0);
									return RobotAPI.ROBOT_OK;
								}else{
									myRobotAPI.setDisplayString("P-espero");
									myRobotAPI.setSteerHeading(myRobotAPI.getBall().t);
									myRobotAPI.setSpeed(0.0);
									return RobotAPI.ROBOT_OK;
								}
							}
				}
				}else{//espera
					myRobotAPI.setDisplayString("P-otro campo");
					myRobotAPI.setSteerHeading(myRobotAPI.getOurGoal().t);
					myRobotAPI.setSpeed(1.0);
					return RobotAPI.ROBOT_OK;
				}
			}
		
		
		}
		return RobotAPI.ROBOT_OK;
	}

}
