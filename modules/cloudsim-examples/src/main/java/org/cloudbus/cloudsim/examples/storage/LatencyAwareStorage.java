package org.cloudbus.cloudsim.examples.storage;

import org.cloudbus.cloudsim.HarddriveStorage;
import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.ParameterException;

public class LatencyAwareStorage extends HarddriveStorage {
    private double readLatency;  // in milliseconds
    private double writeLatency; // in milliseconds
    private Host host;           // Reference to the host/node this storage is associated with

    public LatencyAwareStorage(String name, long capacity, double readLatency, double writeLatency, Host host) throws ParameterException {
        super(name, capacity);
        this.readLatency = readLatency;
        this.writeLatency = writeLatency;
        this.host = host;
    }

    public double getReadLatency() {
        return readLatency;
    }

    public double getWriteLatency() {
        return writeLatency;
    }

    public Host getHost() {
        return host;
    }

    public void setHost(Host host) {
        this.host = host;
    }
}
