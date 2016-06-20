package Equipo;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Message;

/**
 * Created by angel on 18/06/2016.
 */
public class AssistMensaje extends Message {

    Vec2 assistVec;

    public AssistMensaje (Vec2 assistVec) {
        this.assistVec = assistVec;
    }

    public Vec2 getAssistVec() {
        return assistVec;
    }
}
