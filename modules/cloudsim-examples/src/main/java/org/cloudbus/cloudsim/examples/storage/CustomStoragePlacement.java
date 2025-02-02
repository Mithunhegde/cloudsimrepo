package org.cloudbus.cloudsim.examples.storage;

import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Storage;
import java.util.List;
import java.util.Random;
import java.util.Comparator;

/**
 * Implements baseline storage placement policies for benchmarking.
 */
public class CustomStoragePlacement {

    private int roundRobinCounter = 0;

    /**
     * Round-Robin Placement: Distribute storage requests evenly across all nodes.
     */
    public Storage selectStorageRoundRobin(List<Storage> storageList) {
        Storage selectedStorage = storageList.get(roundRobinCounter % storageList.size());
        roundRobinCounter++;
        return selectedStorage;
    }

    /**
     * Least-Full Placement: Allocate storage to the node with the most free space.
     */
    public Storage selectStorageLeastFullWithLatency(List<Storage> storageList, Host sourceHost) {
        Storage bestStorage = null;
        double minLatency = Double.MAX_VALUE;

        for (Storage storage : storageList) {
            if (storage.getAvailableSpace() > 0) {
                LatencyAwareStorage latencyAwareStorage = (LatencyAwareStorage) storage;

                // Calculate total latency (storage latency + network latency)
                double networkLatency = NetworkLatencyCalculator.calculateLatency(sourceHost, latencyAwareStorage.getHost());
                double storageLatency = latencyAwareStorage.getWriteLatency(); // Example: Write operation
                double totalLatency = networkLatency + storageLatency;

                if (totalLatency < minLatency) {
                    minLatency = totalLatency;
                    bestStorage = storage;
                }
            }
        }
        return bestStorage;
    }


    /**
     * Random Placement: Allocate storage to a randomly selected node.
     */
    public Storage selectStorageRandom(List<Storage> storageList) {
        Random random = new Random();
        return storageList.get(random.nextInt(storageList.size()));
    }

    /**
     * First-Fit Placement: Allocate storage to the first node that has sufficient space.
     */
    public Storage selectStorageFirstFit(List<Storage> storageList, long requiredSpace) {
        for (Storage storage : storageList) {
            if (storage.getAvailableSpace() >= requiredSpace) {
                return storage;
            }
        }
        return null; // No suitable storage found
    }
}
