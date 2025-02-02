package org.cloudbus.cloudsim.examples;

import org.cloudbus.cloudsim.*;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
import java.util.*;

public class DataPlacementWithMarkov_NoFrequency {

    public static void main(String[] args) {
        try {
            // Initialize CloudSim
            int numUsers = 1;
            Calendar calendar = Calendar.getInstance();
            boolean traceFlag = false;
            CloudSim.init(numUsers, calendar, traceFlag);

            // Create DataCenter and NoFrequencyCheckBroker
            Datacenter datacenter0 = createDatacenter("Datacenter_0");
            NoFrequencyCheckBroker broker = new NoFrequencyCheckBroker("NoFreqBroker", createMarkovPredictor(),
                    new DataMovementCostEvaluator(50), 0.6);
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

            // Create Cloudlets and record submission times for latency measurement
            List<Cloudlet> cloudletList = new ArrayList<>();
            int cloudletId = 0;
            long length = 40000;
            long fileSize = 300;
            long outputSize = 300;
            UtilizationModel utilizationModel = new UtilizationModelFull();

            // Create several cloudlets to simulate repeated accesses
            for (int i = 0; i < 5; i++) {
                Cloudlet cloudlet = new Cloudlet(cloudletId++, length, pesNumber, fileSize, outputSize,
                        utilizationModel, utilizationModel, utilizationModel);
                cloudlet.setUserId(brokerId);
                broker.setCloudletStartTime(cloudlet.getCloudletId(), CloudSim.clock());
                cloudletList.add(cloudlet);
            }
            broker.submitCloudletList(cloudletList);

            // Start simulation
            CloudSim.startSimulation();
            CloudSim.stopSimulation();

            // After simulation, compute and print latency for each cloudlet
            List<Cloudlet> newList = broker.getCloudletReceivedList();
            for (Cloudlet cl : newList) {
                double submissionTime = broker.getCloudletStartTime(cl.getCloudletId());
                double finishTime = cl.getFinishTime();
                double latency = finishTime - submissionTime;
                System.out.println("Cloudlet " + cl.getCloudletId() + " finished with status " + cl.getStatus());
                System.out.println("Latency for Cloudlet " + cl.getCloudletId() + ": " + latency + " time units.");
            }
            // Print total energy consumption for data movement
            System.out.println("Total energy consumed for data movement (No Frequency Check): " + broker.getTotalEnergyConsumed() + " energy units.");
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Simulation terminated unexpectedly");
        }
    }

    // Create a simple Datacenter
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

    // Create a sample Markov chain predictor with a transition matrix for 2 zones.
    private static MarkovChainPredictorNoFreq createMarkovPredictor() {
        double[][] transitionMatrix = {
                {0.4, 0.6},  // For users in Zone 1: 40% chance to stay, 60% chance to move to Zone 2
                {0.3, 0.7}   // For users in Zone 2: 30% chance to move back, 70% chance to remain
        };
        return new MarkovChainPredictorNoFreq(transitionMatrix);
    }
}

// Markov chain predictor class
class MarkovChainPredictorNoFreq {
    private double[][] transitionMatrix;
    public MarkovChainPredictorNoFreq(double[][] transitionMatrix) {
        this.transitionMatrix = transitionMatrix;
    }
    // Get transition probability from currentZone to targetZone
    public double getTransitionProbability(int currentZone, int targetZone) {
        return transitionMatrix[currentZone][targetZone];
    }
}

// Cost evaluator for data movement
class DataMovementCostEvaluatorNoFreq {
    private double movementCost;
    public DataMovementCostEvaluatorNoFreq(double movementCost) {
        this.movementCost = movementCost;
    }
    public boolean isMovementJustified(double predictedBenefit) {
        return predictedBenefit > movementCost;
    }
}

// Broker class that always moves data when a cloudlet is submitted, ignoring any frequency threshold.
class NoFrequencyCheckBroker extends DatacenterBroker {
    private Map<String, Integer> dataPlacementMap;  // Maps DataItem IDs to zone (0 for Zone 1, 1 for Zone 2)
    private MarkovChainPredictorNoFreq predictor;
    private DataMovementCostEvaluator costEvaluator;
    private double predictionThreshold;   // e.g., 0.6
    // Map to store cloudlet submission times for latency measurement
    private Map<Integer, Double> cloudletStartTimes = new HashMap<>();
    // Energy consumption accumulator
    private double totalEnergyConsumed = 0;
    // Parameters for movement delay calculation
    private double dataItemSize = 1000;
    private Random random = new Random();// Arbitrary units
    private double networkBandwidth =  50 + random.nextDouble() * 100;  ;   // Data units per time unit
    private double distanceFactor = 2.0;     // Multiplier for delay due to distance
    // Energy rate: energy consumed per time unit of data movement delay
    private double energyRate = 5.0;         // Energy units per time unit

    public NoFrequencyCheckBroker(String name, MarkovChainPredictorNoFreq predictor, DataMovementCostEvaluator evaluator, double threshold)
            throws Exception {
        super(name);
        this.dataPlacementMap = new HashMap<>();
        // Initially, DataItem1 is in Zone 0
        dataPlacementMap.put("DataItem1", 0);
        this.predictor = predictor;
        this.costEvaluator = evaluator;
        this.predictionThreshold = threshold;
    }

    // Record cloudlet start time for latency measurement
    public void setCloudletStartTime(int cloudletId, double time) {
        cloudletStartTimes.put(cloudletId, time);
    }

    // Retrieve cloudlet start time
    public double getCloudletStartTime(int cloudletId) {
        return cloudletStartTimes.getOrDefault(cloudletId, 0.0);
    }

    // Retrieve total energy consumed for data movement
    public double getTotalEnergyConsumed() {
        return totalEnergyConsumed;
    }

    // Override submitCloudletList to always check and move data when a cloudlet is submitted
    @Override
    public void submitCloudletList(List<? extends Cloudlet> list) {
        for (Cloudlet cloudlet : list) {
            // For every cloudlet, we always check data movement without a frequency threshold.
            int currentZone = dataPlacementMap.get("DataItem1");
            checkAndMoveData("DataItem1", currentZone, 1);
        }
        super.submitCloudletList(list);
    }

    // Check if data movement is needed based on prediction and cost, then update energy consumption if movement occurs.
    public void checkAndMoveData(String dataItemId, int currentZone, int targetZone) {
        double transitionProb = predictor.getTransitionProbability(currentZone, targetZone);
        System.out.println("Transition probability from Zone " + (currentZone + 1) +
                " to Zone " + (targetZone + 1) + ": " + transitionProb);
        if (transitionProb >= predictionThreshold) {
            double predictedBenefit = transitionProb * 100; // Example scaling factor
            if (costEvaluator.isMovementJustified(predictedBenefit)) {
                double movementDelay = calculateMovementDelay(dataItemSize, networkBandwidth, distanceFactor);
                System.out.println("Simulated data movement delay: " + movementDelay + " time units.");
                double energyConsumed = movementDelay * energyRate;
                totalEnergyConsumed += energyConsumed;
                System.out.println("Energy consumed for moving DataItem1: " + energyConsumed + " energy units.");
                dataPlacementMap.put(dataItemId, targetZone);
                System.out.println("DataItem " + dataItemId + " moved from Zone " + (currentZone + 1) +
                        " to Zone " + (targetZone + 1));
            }
        }
    }

    // Calculate movement delay: delay = (dataSize / bandwidth) * distanceFactor
    public double calculateMovementDelay(double dataSize, double bandwidth, double distanceFactor) {
        return (dataSize / bandwidth) * distanceFactor;
    }
}
