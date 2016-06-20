package Equipo;

import teams.ucmTeam.Message;

/**
 * Created by angel on 20/06/2016.
 */
public class RetreaveMessage extends Message {

    protected long timeStamp;

    public RetreaveMessage(long timeStamp) {
        this.timeStamp = timeStamp;
    }

    public long getTimeStamp() {
        return timeStamp;
    }
}
