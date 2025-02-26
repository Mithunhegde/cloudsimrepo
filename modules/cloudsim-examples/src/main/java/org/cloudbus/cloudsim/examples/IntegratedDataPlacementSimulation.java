//package org.cloudbus.cloudsim.examples;
//
//import org.cloudbus.cloudsim.*;
//import org.cloudbus.cloudsim.core.CloudSim;
//import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
//import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
//import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
//import java.util.*;
//import java.io.FileWriter;
//import java.io.PrintWriter;
//
//public class IntegratedDataPlacementSimulation {
//
//    public static void main(String[] args) {
//        try {
//            // Initialize CloudSim
//            int numUsers = 1;
//            Calendar calendar = Calendar.getInstance();
//            boolean traceFlag = false;
//            CloudSim.init(numUsers, calendar, traceFlag);
//
//            // Create DataCenter
//            Datacenter dc = createDataCenter("DataCenter_Alpha");
//
//            // Create a sample transition matrix for 2 zones.
//            double[][] transMatrix = {
//                    {0.4, 0.6},
//                    {0.3, 0.7}
//            };
//
//            // Build a fuzzy pairwise comparison matrix for two criteria: AccessCount and Recency.
//            // For demonstration:
//            //   AccessCount vs AccessCount: (1, 1, 1)
//            //   AccessCount vs Recency: (2, 3, 4)
//            //   Recency vs AccessCount: (1/4, 1/3, 1/2)
//            //   Recency vs Recency: (1, 1, 1)
//            TriangularFuzzy[][] fuzzyMatrix = new TriangularFuzzy[2][2];
//            fuzzyMatrix[0][0] = new TriangularFuzzy(1, 1, 1);
//            fuzzyMatrix[0][1] = new TriangularFuzzy(2, 3, 4);
//            fuzzyMatrix[1][0] = new TriangularFuzzy(1.0 / 4, 1.0 / 3, 1.0 / 2);
//            fuzzyMatrix[1][1] = new TriangularFuzzy(1, 1, 1);
//            double[] computedWeights = FuzzyAHPProcessor.computeWeights(fuzzyMatrix);
//            System.out.println("Fuzzy AHP Weights: AccessCount = " + computedWeights[0] + ", Recency = " + computedWeights[1]);
//
//            // Create an RL-based Data Broker.
//            // Parameters: predictionThreshold = 0.6, frequencyThreshold = 3,
//            // DQN parameters: epsilon = 0.1, alpha = 0.5, gamma = 0.9, stateBins = 10.
//            // Fuzzy-derived weights used for frequency and recency; hybrid threshold set to 0.5.
//            RLDataBroker broker = new RLDataBroker("RLBroker", transMatrix,
//                    new MigrationCostEvaluator(50), 0.6, 3,
//                    0.1, 0.5, 0.9, 10,
//                    computedWeights[0], computedWeights[1], 0.5);
//            int brokerId = broker.getId();
//
//            // Create two VMs representing two zones.
//            List<Vm> vmList = new ArrayList<>();
//            int vmid = 0;
//            int mips = 250;
//            long size = 10000;
//            int ram = 512;
//            long bw = 1000;
//            int pesNumber = 1;
//            String vmm = "Xen";
//
//            Vm vm1 = new Vm(vmid++, brokerId, mips, pesNumber, ram, bw, size, vmm, new CloudletSchedulerTimeShared());
//            Vm vm2 = new Vm(vmid++, brokerId, mips, pesNumber, ram, bw, size, vmm, new CloudletSchedulerTimeShared());
//            vmList.add(vm1);
//            vmList.add(vm2);
//            broker.submitGuestList(vmList);
//
//            // Create 5 cloudlets (simulate user requests)
//            List<Cloudlet> cloudletList = new ArrayList<>();
//            int cloudletId = 0;
//            long length = 40000;
//            long fileSize = 300;
//            long outputSize = 300;
//            UtilizationModel utilizationModel = new UtilizationModelFull();
//            for (int i = 0; i < 20; i++) {
//                Cloudlet cl = new Cloudlet(cloudletId++, length, pesNumber, fileSize, outputSize,
//                        utilizationModel, utilizationModel, utilizationModel);
//                cl.setUserId(brokerId);
//                broker.recordCloudletSubmission(cl.getCloudletId(), CloudSim.clock());
//                cloudletList.add(cl);
//            }
//            broker.submitCloudletList(cloudletList);
//
//            // Start simulation
//            CloudSim.startSimulation();
//            CloudSim.stopSimulation();
//
//            // Output cloudlet latencies
//            List<Cloudlet> finishedCloudlets = broker.getCloudletReceivedList();
//            for (Cloudlet cl : finishedCloudlets) {
//                double submissionTime = broker.getCloudletSubmissionTime(cl.getCloudletId());
//                double finishTime = cl.getFinishTime();
//                double latency = finishTime - submissionTime;
//                System.out.println("Cloudlet " + cl.getCloudletId() + " finished with status " + cl.getStatus());
//                System.out.println("Latency for Cloudlet " + cl.getCloudletId() + ": " + latency + " time units.");
//            }
//
//            System.out.println("Total energy consumed for data movement: " + broker.getTotalEnergyConsumed() + " energy units.");
//
//        } catch (Exception e) {
//            e.printStackTrace();
//            System.out.println("Simulation terminated unexpectedly");
//        }
//    }
//
//    // Create a simple Datacenter
//    private static Datacenter createDataCenter(String name) {
//        List<Host> hostList = new ArrayList<>();
//        int hostId = 0;
//        int ram = 2048;
//        long storage = 1000000;
//        int bw = 10000;
//        List<Pe> peList = new ArrayList<>();
//        int mips = 1000;
//        peList.add(new Pe(0, new PeProvisionerSimple(mips)));
//        Host host = new Host(hostId, new RamProvisionerSimple(ram), new BwProvisionerSimple(bw),
//                storage, peList, new VmSchedulerTimeShared(peList));
//        hostList.add(host);
//        String arch = "x86";
//        String os = "Linux";
//        String vmm = "Xen";
//        double time_zone = 10.0;
//        double cost = 3.0;
//        double costPerMem = 0.05;
//        double costPerStorage = 0.001;
//        double costPerBw = 0.0;
//        LinkedList<Storage> storageList = new LinkedList<>();
//        DatacenterCharacteristics characteristics = new DatacenterCharacteristics(
//                arch, os, vmm, hostList, time_zone, cost, costPerMem, costPerStorage, costPerBw);
//        Datacenter dc = null;
//        try {
//            dc = new Datacenter(name, characteristics, new VmAllocationPolicySimple(hostList), storageList, 0);
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//        return dc;
//    }
//
//    // Create a sample transition matrix
//    private static double[][] createTransitionMatrix() {
//        double[][] matrix = {
//                {0.4, 0.6},
//                {0.3, 0.7}
//        };
//        return matrix;
//    }
//
//    // ================= Fuzzy AHP Module =================
//    static class TriangularFuzzy {
//        public double l, m, u;
//        public TriangularFuzzy(double l, double m, double u) {
//            this.l = l;
//            this.m = m;
//            this.u = u;
//        }
//        public double defuzzify() {
//            return (l + m + u) / 3.0;
//        }
//        public TriangularFuzzy multiply(TriangularFuzzy other) {
//            return new TriangularFuzzy(this.l * other.l, this.m * other.m, this.u * other.u);
//        }
//        public TriangularFuzzy nthRoot(int n) {
//            return new TriangularFuzzy(Math.pow(this.l, 1.0/n), Math.pow(this.m, 1.0/n), Math.pow(this.u, 1.0/n));
//        }
//        @Override
//        public String toString() {
//            return "(" + l + ", " + m + ", " + u + ")";
//        }
//    }
//
//    static class FuzzyAHPProcessor {
//        public static double[] computeWeights(TriangularFuzzy[][] matrix) {
//            int n = matrix.length;
//            TriangularFuzzy[] geometricMeans = new TriangularFuzzy[n];
//            for (int i = 0; i < n; i++) {
//                TriangularFuzzy product = new TriangularFuzzy(1, 1, 1);
//                for (int j = 0; j < n; j++) {
//                    product = product.multiply(matrix[i][j]);
//                }
//                geometricMeans[i] = product.nthRoot(n);
//            }
//            double[] defuzzified = new double[n];
//            double sum = 0;
//            for (int i = 0; i < n; i++) {
//                defuzzified[i] = geometricMeans[i].defuzzify();
//                sum += defuzzified[i];
//            }
//            double[] weights = new double[n];
//            for (int i = 0; i < n; i++) {
//                weights[i] = defuzzified[i] / sum;
//            }
//            return weights;
//        }
//    }
//
//    // ================= Deep Q-Network Agent (Simplified) =================
//    static class DeepQAgent {
//        private double[][] qTable;
//        private int stateBins;
//        private double epsilon;
//        private double alpha;
//        private double gamma;
//        private Random random;
//
//        public DeepQAgent(int stateBins, double epsilon, double alpha, double gamma) {
//            this.stateBins = stateBins;
//            int totalStates = (int) Math.pow(stateBins, 4); // 4-dimensional state vector
//            this.qTable = new double[totalStates][2]; // 2 actions: 0 (no migration), 1 (migrate)
//            this.epsilon = epsilon;
//            this.alpha = alpha;
//            this.gamma = gamma;
//            this.random = new Random(12345);
//        }
//
//        private int discretize(double value) {
//            int bin = (int) Math.floor(value * stateBins);
//            return Math.min(bin, stateBins - 1);
//        }
//
//        public int getStateIndex(double[] state) {
//            int index = 0;
//            for (int i = 0; i < state.length; i++) {
//                int bin = discretize(state[i]);
//                index = index * stateBins + bin;
//            }
//            return index;
//        }
//
//        public int chooseAction(double[] state) {
//            int stateIndex = getStateIndex(state);
//            if (random.nextDouble() < epsilon) {
//                return random.nextInt(2);
//            } else {
//                return (qTable[stateIndex][0] >= qTable[stateIndex][1]) ? 0 : 1;
//            }
//        }
//
//        public void update(double[] state, int action, double reward, double[] nextState) {
//            int stateIndex = getStateIndex(state);
//            int nextStateIndex = getStateIndex(nextState);
//            double maxNextQ = Math.max(qTable[nextStateIndex][0], qTable[nextStateIndex][1]);
//            qTable[stateIndex][action] = qTable[stateIndex][action] + alpha * (reward + gamma * maxNextQ - qTable[stateIndex][action]);
//        }
//    }
//
//    // ================= Mobility Predictor =================
//    static class MobilityModel {
//        private double[][] transitionMatrix;
//        public MobilityModel(double[][] transitionMatrix) {
//            this.transitionMatrix = transitionMatrix;
//        }
//        public double getTransitionProbability(int currentZone, int targetZone) {
//            return transitionMatrix[currentZone][targetZone];
//        }
//    }
//
//    // ================= Movement Cost Evaluator =================
//    static class MigrationCostEvaluator {
//        private double movementCost;
//        public MigrationCostEvaluator(double movementCost) {
//            this.movementCost = movementCost;
//        }
//        public boolean isMovementJustified(double predictedBenefit) {
//            return predictedBenefit > movementCost;
//        }
//    }
//
//    // ================= RL-based Data Broker =================
//    static class RLDataBroker extends DatacenterBroker {
//        private Map<String, Integer> dataPlacementMap;
//        private Map<String, Integer> dataAccessFrequency;
//        private Map<String, Double> dataLastAccessTime;
//
//        private MobilityModel predictor;
//        private MigrationCostEvaluator costEvaluator;
//        private DeepQAgent agent;
//
//        // Energy consumption accumulator
//        private double totalEnergyConsumed = 0;
//
//        // Parameters for migration delay and energy model
//        private double dataItemSize = 1000;
//        private double distanceFactor = 2.0;
//        private double energyRate = 5.0;
//        // Fixed network bandwidth for simplicity
//        private double networkBandwidth = 100.0;
//
//        // Map to record cloudlet submission times
//        private Map<Integer, Double> cloudletSubmissionTimes = new HashMap<>();
//
//        // Frequency threshold for migration (for comparison purposes)
//        private int frequencyThreshold;
//
//        public RLDataBroker(String name, double[][] transitionMatrix, MigrationCostEvaluator evaluator,
//                            double predictionThreshold, int frequencyThreshold,
//                            double dqnEpsilon, double dqnAlpha, double dqnGamma, int stateBins,
//                            double frequencyWeight, double recencyWeight, double hybridScoreThreshold)
//                throws Exception {
//            super(name);
//            dataPlacementMap = new HashMap<>();
//            dataAccessFrequency = new HashMap<>();
//            dataLastAccessTime = new HashMap<>();
//            // Initially, "DataItem1" is in Zone 0
//            dataPlacementMap.put("DataItem1", 0);
//            dataAccessFrequency.put("DataItem1", 0);
//            dataLastAccessTime.put("DataItem1", 0.0);
//
//            this.predictor = new MobilityModel(transitionMatrix);
//            this.costEvaluator = evaluator;
//            this.frequencyThreshold = frequencyThreshold;
//
//            // Initialize DQN agent with given parameters
//            agent = new DeepQAgent(stateBins, dqnEpsilon, dqnAlpha, dqnGamma);
//        }
//
//        public void recordCloudletSubmission(int cloudletId, double time) {
//            cloudletSubmissionTimes.put(cloudletId, time);
//            updateDataAccess("DataItem1", time);
//            int currentZone = dataPlacementMap.get("DataItem1");
//            double transitionProb = predictor.getTransitionProbability(currentZone, 1);
//            // Normalize frequency assuming a max frequency of 10
//            double normalizedFreq = (double) dataAccessFrequency.get("DataItem1") / 10.0;
//            double currentTime = CloudSim.clock();
//            double recency = 1.0 / (currentTime - dataLastAccessTime.get("DataItem1") + 1);
//            double[] state = new double[] { normalizedFreq, recency, (double) currentZone, transitionProb };
//            int action = agent.chooseAction(state);
//            System.out.println("DQN Agent chose action: " + action + " for state: " + Arrays.toString(state));
//            double reward = 0;
//            if (action == 1) {
//                double movementDelay = (dataItemSize / networkBandwidth) * distanceFactor;
//                double energyConsumed = movementDelay * energyRate;
//                totalEnergyConsumed += energyConsumed;
//                System.out.println("Migration performed. Movement delay: " + movementDelay +
//                        ", Energy consumed: " + energyConsumed);
//                dataPlacementMap.put("DataItem1", 1);
//                reward = -energyConsumed;
//            } else {
//                reward = 0;
//            }
//            double[] nextState = state;  // For simplicity, use the same state.
//            agent.update(state, action, reward, nextState);
//        }
//
//        private void updateDataAccess(String dataItemId, double time) {
//            int freq = dataAccessFrequency.getOrDefault(dataItemId, 0);
//            dataAccessFrequency.put(dataItemId, freq + 1);
//            dataLastAccessTime.put(dataItemId, time);
//        }
//
//        public double getCloudletSubmissionTime(int cloudletId) {
//            return cloudletSubmissionTimes.getOrDefault(cloudletId, 0.0);
//        }
//
//        @Override
//        public void submitCloudletList(List<? extends Cloudlet> list) {
//            for (Cloudlet cloudlet : list) {
//                recordCloudletSubmission(cloudlet.getCloudletId(), CloudSim.clock());
//            }
//            super.submitCloudletList(list);
//        }
//
//        public double getTotalEnergyConsumed() {
//            return totalEnergyConsumed;
//        }
//    }
//}

















package org.cloudbus.cloudsim.examples;

import org.cloudbus.cloudsim.*;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
import java.util.*;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.nn.conf.MultiLayerConfiguration;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.layers.OutputLayer;
import org.deeplearning4j.optimize.api.IterationListener;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.lossfunctions.LossFunctions;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

public class IntegratedDataPlacementSimulation {

    public static void main(String[] args) {
        try {
            // Initialize CloudSim
            int numUsers = 1;
            Calendar calendar = Calendar.getInstance();
            boolean traceFlag = false;
            CloudSim.init(numUsers, calendar, traceFlag);

            // Create DataCenter
            Datacenter dc = createDataCenter("DataCenter_Alpha");

            // Create a sample transition matrix for 2 zones.
            double[][] transMatrix = {
                    {0.4, 0.6},
                    {0.3, 0.7}
            };

            // Build a fuzzy pairwise comparison matrix for two criteria: AccessCount and Recency.
            TriangularFuzzy[][] fuzzyMatrix = new TriangularFuzzy[2][2];
            fuzzyMatrix[0][0] = new TriangularFuzzy(1, 1, 1);
            fuzzyMatrix[0][1] = new TriangularFuzzy(2, 3, 4);
            fuzzyMatrix[1][0] = new TriangularFuzzy(1.0 / 4, 1.0 / 3, 1.0 / 2);
            fuzzyMatrix[1][1] = new TriangularFuzzy(1, 1, 1);
            double[] computedWeights = FuzzyAHPProcessor.computeWeights(fuzzyMatrix);
            System.out.println("Fuzzy AHP Weights: AccessCount = " + computedWeights[0] + ", Recency = " + computedWeights[1]);

            // Create an RL-based Data Broker using the DL4J DQN agent.
            // Parameters: predictionThreshold = 0.6, frequencyThreshold = 3,
            // DQN parameters: epsilon = 0.1, alpha = 0.5, gamma = 0.9, stateBins = 10,
            // Fuzzy-derived weights used for frequency and recency; hybrid threshold = 0.5.
            RLDataBroker broker = new RLDataBroker("RLBroker", transMatrix,
                    new MigrationCostEvaluator(50), 0.6, 3,
                    0.1, 0.5, 0.9, 10,
                    computedWeights[0], computedWeights[1], 0.5);
            int brokerId = broker.getId();

            // Create two VMs representing two zones.
            List<Vm> vmList = new ArrayList<>();
            int vmid = 0;
            int mips = 250;
            long size = 10000;
            int ram = 512;
            long bw = 1000;
            int pesNumber = 1;
            String vmm = "Xen";

            Vm vm1 = new Vm(vmid++, brokerId, mips, pesNumber, ram, bw, size, vmm, new CloudletSchedulerTimeShared());
            Vm vm2 = new Vm(vmid++, brokerId, mips, pesNumber, ram, bw, size, vmm, new CloudletSchedulerTimeShared());
            vmList.add(vm1);
            vmList.add(vm2);
            broker.submitGuestList(vmList);

            // Create 20 cloudlets (simulate user requests)
            List<Cloudlet> cloudletList = new ArrayList<>();
            int cloudletId = 0;
            long length = 40000;
            long fileSize = 300;
            long outputSize = 300;
            UtilizationModel utilizationModel = new UtilizationModelFull();
            for (int i = 0; i < 20; i++) {
                Cloudlet cl = new Cloudlet(cloudletId++, length, pesNumber, fileSize, outputSize,
                        utilizationModel, utilizationModel, utilizationModel);
                cl.setUserId(brokerId);
                broker.recordCloudletSubmission(cl.getCloudletId(), CloudSim.clock());
                cloudletList.add(cl);
            }
            broker.submitCloudletList(cloudletList);

            // Start simulation
            CloudSim.startSimulation();
            CloudSim.stopSimulation();

            // Output cloudlet latencies
            List<Cloudlet> finishedCloudlets = broker.getCloudletReceivedList();
            for (Cloudlet cl : finishedCloudlets) {
                double submissionTime = broker.getCloudletSubmissionTime(cl.getCloudletId());
                double finishTime = cl.getFinishTime();
                double latency = finishTime - submissionTime;
                System.out.println("Cloudlet " + cl.getCloudletId() + " finished with status " + cl.getStatus());
                System.out.println("Latency for Cloudlet " + cl.getCloudletId() + ": " + latency + " time units.");
            }

            System.out.println("Total energy consumed for data movement: " + broker.getTotalEnergyConsumed() + " energy units.");

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Simulation terminated unexpectedly");
        }
    }

    // Create a simple Datacenter.
    private static Datacenter createDataCenter(String name) {
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
        Datacenter dc = null;
        try {
            dc = new Datacenter(name, characteristics, new VmAllocationPolicySimple(hostList), storageList, 0);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return dc;
    }

    private static double[][] createTransitionMatrix() {
        double[][] matrix = {
                {0.4, 0.6},
                {0.3, 0.7}
        };
        return matrix;
    }

    // ================= Fuzzy AHP Module =================
    static class TriangularFuzzy {
        public double l, m, u;
        public TriangularFuzzy(double l, double m, double u) {
            this.l = l;
            this.m = m;
            this.u = u;
        }
        public double defuzzify() {
            return (l + m + u) / 3.0;
        }
        public TriangularFuzzy multiply(TriangularFuzzy other) {
            return new TriangularFuzzy(this.l * other.l, this.m * other.m, this.u * other.u);
        }
        public TriangularFuzzy nthRoot(int n) {
            return new TriangularFuzzy(Math.pow(this.l, 1.0 / n), Math.pow(this.m, 1.0 / n), Math.pow(this.u, 1.0 / n));
        }
        @Override
        public String toString() {
            return "(" + l + ", " + m + ", " + u + ")";
        }
    }

    static class FuzzyAHPProcessor {
        public static double[] computeWeights(TriangularFuzzy[][] matrix) {
            int n = matrix.length;
            TriangularFuzzy[] geometricMeans = new TriangularFuzzy[n];
            for (int i = 0; i < n; i++) {
                TriangularFuzzy product = new TriangularFuzzy(1, 1, 1);
                for (int j = 0; j < n; j++) {
                    product = product.multiply(matrix[i][j]);
                }
                geometricMeans[i] = product.nthRoot(n);
            }
            double[] defuzzified = new double[n];
            double sum = 0;
            for (int i = 0; i < n; i++) {
                defuzzified[i] = geometricMeans[i].defuzzify();
                sum += defuzzified[i];
            }
            double[] weights = new double[n];
            for (int i = 0; i < n; i++) {
                weights[i] = defuzzified[i] / sum;
            }
            return weights;
        }
    }

    // ================= DL4J-based Deep Q-Network Agent =================
    static class DL4JDQNAgent {
        private MultiLayerNetwork qNetwork;
        private double epsilon;
        private double alpha;
        private double gamma;
        private int stateSize;
        private int actionSize;
        private Random random;

        public DL4JDQNAgent(int stateSize, int actionSize, double epsilon, double alpha, double gamma) {
            this.stateSize = stateSize;
            this.actionSize = actionSize;
            this.epsilon = epsilon;
            this.alpha = alpha;
            this.gamma = gamma;
            this.random = new Random(12345); // Fixed seed for reproducibility
            this.qNetwork = buildNetwork(stateSize, actionSize);
        }


        private MultiLayerNetwork buildNetwork(int inputSize, int outputSize) {
            MultiLayerConfiguration conf = new NeuralNetConfiguration.Builder()
                    .seed(12345)
                    .updater(new org.nd4j.linalg.learning.config.Nesterovs(0.001, 0.9))
                    .list()
                    .layer(0, new DenseLayer.Builder().nIn(inputSize).nOut(16)
                            .activation(Activation.RELU)
                            .build())
                    .layer(1, new DenseLayer.Builder().nIn(16).nOut(16)
                            .activation(Activation.RELU)
                            .build())
                    .layer(2, new OutputLayer.Builder(LossFunctions.LossFunction.MSE)
                            .activation(Activation.IDENTITY)
                            .nIn(16).nOut(outputSize)
                            .build())
                    .build();
            MultiLayerNetwork model = new MultiLayerNetwork(conf);
            model.init();
            return model;
        }

        // Choose an action using ε-greedy policy.
        // Convert the state vector to a 2D row vector.
        public int chooseAction(double[] state) {
            INDArray input = Nd4j.create(state, new int[]{1, state.length});
            if (random.nextDouble() < epsilon) {
                return random.nextInt(actionSize);
            } else {
                INDArray output = qNetwork.output(input, false);
                // Argmax along dimension 1 to get the index of the best action
                return Nd4j.argMax(output, 1).getInt(0);
            }
        }

        // Update the Q-network using a single sample (state, action, reward, nextState)
        public void update(double[] state, int action, double reward, double[] nextState) {
            INDArray stateArray = Nd4j.create(state, new int[]{1, state.length});
            INDArray nextStateArray = Nd4j.create(nextState, new int[]{1, nextState.length});
            INDArray qValues = qNetwork.output(stateArray, false);
            INDArray qValuesNext = qNetwork.output(nextStateArray, false);
            double maxNextQ = qValuesNext.maxNumber().doubleValue();
            double targetQ = reward + gamma * maxNextQ;
            // Update the Q-value for the chosen action. We use row 0 since input is 1xN.
            qValues.putScalar(0, action, targetQ);
            qNetwork.fit(stateArray, qValues);
        }
    }


    // ================= RL-based Data Broker using DL4JDQNAgent =================
    static class RLDataBroker extends DatacenterBroker {
        private Map<String, Integer> dataPlacementMap;
        private Map<String, Integer> dataAccessFrequency;
        private Map<String, Double> dataLastAccessTime;

        private MobilityModel predictor;
        private MigrationCostEvaluator costEvaluator;
        private DL4JDQNAgent agent;

        // Energy consumption accumulator
        private double totalEnergyConsumed = 0;

        // Parameters for migration delay and energy model
        private double dataItemSize = 1000;
        private double distanceFactor = 2.0;
        private double energyRate = 5.0;
        // Fixed network bandwidth for simplicity
        //private double networkBandwidth = 100.0;

        private Random random = new Random();// Arbitrary units
        private double networkBandwidth =  50 + random.nextDouble() * 100;

        // Map to record cloudlet submission times
        private Map<Integer, Double> cloudletSubmissionTimes = new HashMap<>();

        // Frequency threshold for migration (for comparison)
        private int frequencyThreshold;

        private double frequencyWeight=0;
        private double recencyWeight=0;


        public RLDataBroker(String name, double[][] transitionMatrix, MigrationCostEvaluator evaluator,
                            double predictionThreshold, int frequencyThreshold,
                            double dqnEpsilon, double dqnAlpha, double dqnGamma, int stateBins,
                            double frequencyWeight, double recencyWeight, double hybridScoreThreshold)
                throws Exception {
            super(name);
            dataPlacementMap = new HashMap<>();
            dataAccessFrequency = new HashMap<>();
            dataLastAccessTime = new HashMap<>();
            dataPlacementMap.put("DataItem1", 0);
            dataAccessFrequency.put("DataItem1", 0);
            dataLastAccessTime.put("DataItem1", 0.0);

            this.predictor = new MobilityModel(transitionMatrix);
            this.costEvaluator = evaluator;
            this.frequencyThreshold = frequencyThreshold;
            this.frequencyWeight=frequencyWeight;
            this.recencyWeight=recencyWeight;

            // Initialize DL4J DQN agent: state vector of size 4, action space of size 2.
            agent = new DL4JDQNAgent(4, 2, dqnEpsilon, dqnAlpha, dqnGamma);
        }

        public void recordCloudletSubmission(int cloudletId, double time) {
            cloudletSubmissionTimes.put(cloudletId, time);
            updateDataAccess("DataItem1", time);
            int currentZone = dataPlacementMap.get("DataItem1");
            double transitionProb = predictor.getTransitionProbability(currentZone, 1);
            double normalizedFreq = (double) dataAccessFrequency.get("DataItem1") / 10.0*frequencyWeight;
            double currentTime = CloudSim.clock();
            double recency = 1.0 / (currentTime - dataLastAccessTime.get("DataItem1") + 1)*recencyWeight;
            double[] state = new double[] { normalizedFreq, recency, (double) currentZone, transitionProb };
            int action = agent.chooseAction(state);
            System.out.println("DQN Agent chose action: " + action + " for state: " + Arrays.toString(state));
            double reward = 0;
            if (action == 1) {
                double movementDelay = (dataItemSize / networkBandwidth) * distanceFactor;
                double energyConsumed = movementDelay * energyRate;
                totalEnergyConsumed += energyConsumed;
                System.out.println("Migration performed. Movement delay: " + movementDelay +
                        ", Energy consumed: " + energyConsumed);
                dataPlacementMap.put("DataItem1", 1);
                reward = -energyConsumed;
            } else {
                reward = 0;
            }
            double[] nextState = state;  // For simplicity, using same state.
            agent.update(state, action, reward, nextState);
        }

        private void updateDataAccess(String dataItemId, double time) {
            int freq = dataAccessFrequency.getOrDefault(dataItemId, 0);
            dataAccessFrequency.put(dataItemId, freq + 1);
            dataLastAccessTime.put(dataItemId, time);
        }

        public double getCloudletSubmissionTime(int cloudletId) {
            return cloudletSubmissionTimes.getOrDefault(cloudletId, 0.0);
        }

        @Override
        public void submitCloudletList(List<? extends Cloudlet> list) {
            for (Cloudlet cloudlet : list) {
                recordCloudletSubmission(cloudlet.getCloudletId(), CloudSim.clock());
            }
            super.submitCloudletList(list);
        }

        public double getTotalEnergyConsumed() {
            return totalEnergyConsumed;
        }
    }

    // ================= Mobility Predictor (Renamed) =================
    static class MobilityModel {
        private double[][] transitionMatrix;
        public MobilityModel(double[][] transitionMatrix) {
            this.transitionMatrix = transitionMatrix;
        }
        public double getTransitionProbability(int currentZone, int targetZone) {
            return transitionMatrix[currentZone][targetZone];
        }
    }

    // ================= Movement Cost Evaluator (Renamed) =================
    static class MigrationCostEvaluator {
        private double movementCost;
        public MigrationCostEvaluator(double movementCost) {
            this.movementCost = movementCost;
        }
        public boolean isMovementJustified(double predictedBenefit) {
            return predictedBenefit > movementCost;
        }
    }
}

