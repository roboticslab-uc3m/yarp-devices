
/*import yarp.Network;
import yarp.Port;
import yarp.Bottle;
import yarp.Vocab;*/

import yarp.*;

class CartesianClient {

    private Port p;
    private int VOCAB_STAT, VOCAB_INV, VOCAB_MOVJ, VOCAB_MOVL, VOCAB_MY_STOP, VOCAB_WAIT;

    public CartesianClient() {
        VOCAB_STAT = Vocab.encode("stat");
        VOCAB_INV = Vocab.encode("inv");
        VOCAB_MOVJ = Vocab.encode("movj");
        VOCAB_MOVL = Vocab.encode("movl");
        VOCAB_MY_STOP = Vocab.encode("stop");
        VOCAB_WAIT = Vocab.encode("wait");
	    System.out.println("Java CartesianClient contructor");
        System.out.flush();
    }

    public boolean open(String prefix) {
        String clientPort = prefix;
        clientPort += "/cartesianClient/rpc:o";
        p = new Port();
        p.open(clientPort);
        String serverPort = prefix;
        serverPort += "/cartesianServer/rpc:i";
        p.addOutput(serverPort);
        return true;
    }

    public boolean close() {
        p.interrupt();
        p.close();
        return true;
    }

    public boolean stat(DVector res) {
        Bottle miOutput = new Bottle();
        Bottle miInput = new Bottle();
        miOutput.clear();
        miOutput.addVocab(VOCAB_STAT);
        p.write(miOutput, miInput);
        Bottle data = miInput.get(1).asList();
        res.clear();
        for (int elem=0; elem<data.size(); elem++)
            res.add(data.get(elem).asDouble());
        return true;
    }

    public boolean stop() {
        Bottle miOutput = new Bottle();
        Bottle miInput = new Bottle();
        miOutput.clear();
        miOutput.addVocab(VOCAB_MY_STOP);
        p.write(miOutput, miInput);
        return true;
    }

    public boolean inv(double[] xd, DVector res) {
        Bottle miOutput = new Bottle();
        Bottle miInput = new Bottle();
        miOutput.clear();
        miOutput.addVocab(VOCAB_INV);
        Bottle dBottle = new Bottle();
        dBottle = miOutput.addList();
        for (int elem=0; elem<xd.length; elem++)
            dBottle.addDouble(xd[elem]);
        p.write(miOutput, miInput);
        Bottle data = miInput.get(0).asList();
        res.clear();
        for (int elem=0; elem<data.size(); elem++)
            res.add(data.get(elem).asDouble());
        return true;
    }

    public boolean movj(double[] xd) {
        Bottle miOutput = new Bottle();
        Bottle miInput = new Bottle();
        miOutput.clear();
        miOutput.addVocab(VOCAB_MOVJ);
        Bottle dBottle = new Bottle();
        dBottle = miOutput.addList();
        for (int elem=0; elem<xd.length; elem++)
            dBottle.addDouble(xd[elem]);
        p.write(miOutput, miInput);
        return true;
    }

    public boolean movl(double[] xd) {
        Bottle miOutput = new Bottle();
        Bottle miInput = new Bottle();
        miOutput.clear();
        miOutput.addVocab(VOCAB_MOVL);
        Bottle dBottle = new Bottle();
        dBottle = miOutput.addList();
        for (int elem=0; elem<xd.length; elem++)
            dBottle.addDouble(xd[elem]);
        p.write(miOutput, miInput);
        return true;
    }

    public boolean Wait() {
        Bottle miOutput = new Bottle();
        Bottle miInput = new Bottle();
        miOutput.clear();
        miOutput.addVocab(VOCAB_WAIT);
        p.write(miOutput, miInput);
        return true;
    }
}

