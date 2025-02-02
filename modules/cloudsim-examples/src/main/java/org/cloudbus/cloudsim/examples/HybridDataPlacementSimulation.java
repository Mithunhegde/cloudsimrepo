package org.cloudbus.cloudsim.examples;

import org.cloudbus.cloudsim.*;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
import java.util.*;

public class HybridDataPlacementSimulation {

    public static void main(String[] args) {
        try {
            // Initialize CloudSim
            int numUsers = 1;
            Calendar calendar = Calendar.getInstance();
            boolean traceFlag = false;
            CloudSim.init(numUsers, calendar, traceFlag);

            // Create DataCenter and Hybrid Broker
            Datacenter datacenter0 = createDatacenter("Datacenter_0");
            // Parameters for HybridDataBroker:
            // predictionThreshold = 0.6, frequencyWeight = 0.5, recencyWeight = 0.5, hybridScoreThreshold = 0.5
            HybridDataBroker broker = new HybridDataBroker("HybridBroker", createTransitionMatrix(),
                    new MovementCostEvaluator(50), 0.6, 0.5, 0.5, 0.5);
            int brokerId = broker.getId();

            // Create two VMs representing two zones:
            List<Vm> vmList = new ArrayList<>();
            int vmid = 0;
            int mips = 250;
            long size = 10000;
            int ram = 512;
            long bw = 1000;
            int pesNumber = 1;
            String vmm = "Xen";

            // VM for Zone 1
            Vm vm1 = new Vm(vmid++, brokerId, mips, pesNumber, ram, bw, size, vmm, new CloudletSchedulerTimeShared());
            // VM for Zone 2
            Vm vm2 = new Vm(vmid++, brokerId, mips, pesNumber, ram, bw, size, vmm, new CloudletSchedulerTimeShared());
            vmList.add(vm1);
            vmList.add(vm2);
            broker.submitGuestList(vmList);

            // Create Cloudlets (simulate multiple user requests)
            List<Cloudlet> cloudletList = new ArrayList<>();
            int cloudletId = 0;
            long length = 40000;
            long fileSize = 300;
            long outputSize = 300;
            UtilizationModel utilizationModel = new UtilizationModelFull();

            // Create 5 cloudlets
            for (int i = 0; i < 5; i++) {
                Cloudlet cloudlet = new Cloudlet(cloudletId++, length, pesNumber, fileSize, outputSize,
                        utilizationModel, utilizationModel, utilizationModel);
                cloudlet.setUserId(brokerId);
                // Record submission time for latency measurement and update data access metrics.
                broker.recordCloudletSubmission(cloudlet.getCloudletId(), CloudSim.clock());
                cloudletList.add(cloudlet);
            }
            broker.submitCloudletList(cloudletList);

            // Start simulation
            CloudSim.startSimulation();
            CloudSim.stopSimulation();

            // Print results: latency per cloudlet and total energy consumption.
            List<Cloudlet> newList = broker.getCloudletReceivedList();
            for (Cloudlet cl : newList) {
                double submissionTime = broker.getCloudletSubmissionTime(cl.getCloudletId());
                double finishTime = cl.getFinishTime();
                double latency = finishTime - submissionTime;
                System.out.println("Cloudlet " + cl.getCloudletId() + " latency: " + latency + " time units.");
            }
            System.out.println("Total energy consumed for data movement: " + broker.getTotalEnergyConsumed() + " energy units.");

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Simulation terminated unexpectedly");
        }
    }

    // Create a simple Datacenter (same as previous examples)
    private static Datacenter createDatacenter(String name) {
        List<Host> hostList = new ArrayList<>();
        int hostId = 0;
        int ram = 2048;
        long storage = 1000000;
        int bw = 10000;
        List<Pe> peList = new ArrayList<>();
        int mips = 1000;
        peList.add(new Pe(0, new PeProvisionerSimple(mips)));
        Host host = new Host(hostId, new RamProvisionerSimple(ram), new BwProvisionerSimple(bw),
                storage, peList, new VmSchedulerTimeShared(peList));
        hostList.add(host);
        String arch = "x86";
        String os = "Linux";
        String vmm = "Xen";
        double time_zone = 10.0;
        double cost = 3.0;
        double costPerMem = 0.05;
        double costPerStorage = 0.001;
        double costPerBw = 0.0;
        LinkedList<Storage> storageList = new LinkedList<>();
        DatacenterCharacteristics characteristics = new DatacenterCharacteristics(
                arch, os, vmm, hostList, time_zone, cost, costPerMem, costPerStorage, costPerBw);
        Datacenter datacenter = null;
        try {
            datacenter = new Datacenter(name, characteristics, new VmAllocationPolicySimple(hostList), storageList, 0);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return datacenter;
    }

    // Create a sample transition matrix for a Markov chain with 2 zones.
    // Row 0: [Probability to stay in Zone 1, Probability to move to Zone 2]
    // Row 1: [Probability to move to Zone 1, Probability to stay in Zone 2]
    private static double[][] createTransitionMatrix() {
        double[][] matrix = {
                {0.4, 0.6},
                {0.3, 0.7}
        };
        return matrix;
    }
}

// HybridDataBroker: extends DatacenterBroker and uses hybrid metrics (frequency + recency) with dynamic network conditions.
class HybridDataBroker extends DatacenterBroker {

    // Maps to track data placement, access frequency, and last access time
    private Map<String, Integer> dataPlacementMap;
    private Map<String, Integer> dataAccessFrequency;
    private Map<String, Double> dataLastAccessTime;

    private MobilityPredictor predictor;
    private MovementCostEvaluator costEvaluator;

    // Thresholds and weights for decision-making:
    private double predictionThreshold;    // e.g., 0.6
    private double frequencyWeight;        // Weight for frequency in hybrid score (alpha)
    private double recencyWeight;          // Weight for recency in hybrid score (1 - alpha)
    private double hybridScoreThreshold;   // Minimum hybrid score required to consider movement

    // Energy consumption accumulator
    private double totalEnergyConsumed = 0;

    // Parameters for movement delay calculation and energy model
    private double dataItemSize = 1000;     // Arbitrary units
    private double distanceFactor = 2.0;    // Multiplier for delay due to distance
    private double energyRate = 5.0;        // Energy units per time unit
    private Random rand = new Random();

    // Map to store cloudlet submission times for latency measurement
    private Map<Integer, Double> cloudletSubmissionTimes = new HashMap<>();

    public HybridDataBroker(String name, double[][] transitionMatrix, MovementCostEvaluator evaluator,
                            double predictionThreshold, double frequencyWeight, double recencyWeight, double hybridScoreThreshold)
            throws Exception {
        super(name);
        // Initialize maps for one data item "DataItem1"
        dataPlacementMap = new HashMap<>();
        dataAccessFrequency = new HashMap<>();
        dataLastAccessTime = new HashMap<>();
        dataPlacementMap.put("DataItem1", 0);  // Initially in Zone 0
        dataAccessFrequency.put("DataItem1", 0);
        dataLastAccessTime.put("DataItem1", 0.0);

        this.predictor = new MobilityPredictor(transitionMatrix);
        this.costEvaluator = evaluator;
        this.predictionThreshold = predictionThreshold;
        this.frequencyWeight = frequencyWeight;
        this.recencyWeight = recencyWeight;
        this.hybridScoreThreshold = hybridScoreThreshold;
    }

    // Record cloudlet submission time and update access metrics for a data item.
    public void recordCloudletSubmission(int cloudletId, double time) {
        cloudletSubmissionTimes.put(cloudletId, time);
        updateDataAccess("DataItem1", time);
        // After updating, decide if data should be moved.
        int currentZone = dataPlacementMap.get("DataItem1");
        checkAndMoveData("DataItem1", currentZone, 1); // For demonstration, target zone fixed to 1
    }

    public double getCloudletSubmissionTime(int cloudletId) {
        return cloudletSubmissionTimes.getOrDefault(cloudletId, 0.0);
    }

    // Update frequency and recency for a data item.
    private void updateDataAccess(String dataItemId, double time) {
        int freq = dataAccessFrequency.getOrDefault(dataItemId, 0);
        dataAccessFrequency.put(dataItemId, freq + 1);
        dataLastAccessTime.put(dataItemId, time);
    }

    // Compute hybrid score: combines frequency and recency.
    // For recency, we use an inverted value: recencyScore = 1 / (currentTime - lastAccess + 1).
    private double computeHybridScore(String dataItemId, double currentTime) {
        int freq = dataAccessFrequency.getOrDefault(dataItemId, 0);
        double lastAccess = dataLastAccessTime.getOrDefault(dataItemId, 0.0);
        double recencyScore = 1.0 / (currentTime - lastAccess + 1);
        double score = frequencyWeight * freq + recencyWeight * recencyScore;
        System.out.println("Frequency: " + freq + ", RecencyScore: " + recencyScore + ", HybridScore: " + score);
        return score;
    }

    // Simulate dynamic network bandwidth as a random value between min and max (e.g., 50 and 150)
    private double getCurrentBandwidth() {
        return 50 + rand.nextDouble() * 100;
    }

    // Check if data should be moved based on hybrid score, Markov prediction, and cost.
    public void checkAndMoveData(String dataItemId, int currentZone, int targetZone) {
        double currentTime = CloudSim.clock();
        double hybridScore = computeHybridScore(dataItemId, currentTime);
        if (hybridScore >= hybridScoreThreshold) {
            double transitionProb = predictor.getTransitionProbability(currentZone, targetZone);
            System.out.println("Transition probability from Zone " + (currentZone + 1) +
                    " to Zone " + (targetZone + 1) + ": " + transitionProb);
            if (transitionProb >= predictionThreshold) {
                double predictedBenefit = transitionProb * 100; // Example scaling factor
                if (costEvaluator.isMovementJustified(predictedBenefit)) {
                    // Use dynamic bandwidth for delay calculation
                    double currentBandwidth = getCurrentBandwidth();
                    double movementDelay = (dataItemSize / currentBandwidth) * distanceFactor;
                    System.out.println("Current bandwidth: " + currentBandwidth + " -> Movement delay: " + movementDelay);
                    double energyConsumed = movementDelay * energyRate;
                    totalEnergyConsumed += energyConsumed;
                    System.out.println("Energy consumed for moving " + dataItemId + ": " + energyConsumed);
                    dataPlacementMap.put(dataItemId, targetZone);
                    System.out.println("DataItem " + dataItemId + " moved from Zone " + (currentZone + 1) +
                            " to Zone " + (targetZone + 1));
                }
            }
        } else {
            System.out.println("Hybrid score (" + hybridScore + ") below threshold (" + hybridScoreThreshold + "); no movement.");
        }
    }

    public double getTotalEnergyConsumed() {
        return totalEnergyConsumed;
    }
}

// MobilityPredictor: a simple Markov chain predictor.
class MobilityPredictor {
    private double[][] transitionMatrix;
    public MobilityPredictor(double[][] transitionMatrix) {
        this.transitionMatrix = transitionMatrix;
    }
    // Get transition probability from currentZone to targetZone.
    public double getTransitionProbability(int currentZone, int targetZone) {
        return transitionMatrix[currentZone][targetZone];
    }
}

// MovementCostEvaluator: evaluates if data movement is justified based on predicted benefit.
class MovementCostEvaluator {
    private double movementCost;
    public MovementCostEvaluator(double movementCost) {
        this.movementCost = movementCost;
    }
    public boolean isMovementJustified(double predictedBenefit) {
        return predictedBenefit > movementCost;
    }
}
