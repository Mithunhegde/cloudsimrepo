package org.cloudbus.cloudsim.examples.storage;

import org.cloudbus.cloudsim.Host;

public class NetworkLatencyCalculator {
    public static double calculateLatency(Host sourceHost, Host targetHost) {
        // Example: Network latency based on the difference in host IDs
        int distance = Math.abs(sourceHost.getId() - targetHost.getId());
        double latencyPerUnit = 2.0; // Example: 2ms per unit distance
        return distance * latencyPerUnit;
    }
}
