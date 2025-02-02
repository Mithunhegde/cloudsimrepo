package org.cloudbus.cloudsim.examples;

import org.cloudbus.cloudsim.*;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.examples.storage.CustomDatacenter;
import org.cloudbus.cloudsim.examples.storage.CustomStoragePlacement;
import org.cloudbus.cloudsim.examples.storage.LatencyAwareStorage;
import org.cloudbus.cloudsim.network.datacenter.NetworkHost;
import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

public class CloudSimStoragePlacementExample {

    public static void main(String[] args) {
        Log.printLine("Starting CloudSimStoragePlacementExample...");

        try {
            int num_user = 1; // number of cloud users
            Calendar calendar = Calendar.getInstance();
            boolean trace_flag = false; // mean trace events

            // Initialize the CloudSim library
            CloudSim.init(num_user, calendar, trace_flag);

            // Create Datacenter
            Datacenter datacenter = createDatacenter();

            // Create CustomStoragePlacement instance
            CustomStoragePlacement placement = new CustomStoragePlacement();

            // Get Storage List from Datacenter
            List<Storage> storageList = ((CustomDatacenter) datacenter).getStorageList();
            Host sourceHost = (Host) datacenter.getHostList().get(0);

            // Test Round-Robin Placement
            Storage selectedStorage = placement.selectStorageRoundRobin(storageList);
            Log.printLine("Round-Robin: Selected Storage: " + selectedStorage.getName());

            // Test Least-Full Placement
            selectedStorage = placement.selectStorageLeastFullWithLatency(storageList,sourceHost);
            Log.printLine("Least-Full: Selected Storage: " + selectedStorage.getName());

            // Test Random Placement
            selectedStorage = placement.selectStorageRandom(storageList);
            Log.printLine("Random: Selected Storage: " + selectedStorage.getName());

            // Test First-Fit Placement
            long requiredSpace = 100000; // Example file size
            selectedStorage = placement.selectStorageFirstFit(storageList, requiredSpace);
            if (selectedStorage != null) {
                Log.printLine("First-Fit: Selected Storage: " + selectedStorage.getName());
            } else {
                Log.printLine("First-Fit: No suitable storage found!");
            }

            Log.printLine("CloudSimStoragePlacementExample finished!");
        } catch (Exception e) {
            e.printStackTrace();
            Log.printLine("The simulation has been terminated due to an unexpected error");
        }
    }

    private static Datacenter createDatacenter() throws ParameterException {
        // Create a list of storage devices
        List<Storage> storageList = new ArrayList<>();
        List<Host> hostList = CreateHosts();
        storageList.add(new LatencyAwareStorage("Storage1", 1000000, 5.0, 10.0, hostList.get(0))); // Host 0
        storageList.add(new LatencyAwareStorage("Storage2", 500000, 3.0, 8.0, hostList.get(1)));  // Host 1
        storageList.add(new LatencyAwareStorage("Storage3", 750000, 7.0, 12.0, hostList.get(2)));

        // Create a list of hosts (required by VmAllocationPolicySimple)


        // Create DatacenterCharacteristics
        String arch = "x86";
        String os = "Linux";
        String vmm = "Xen";
        double time_zone = 10.0;
        double cost = 3.0;
        double costPerMem = 0.05;
        double costPerStorage = 0.001;
        double costPerBw = 0.0;

        DatacenterCharacteristics characteristics = new DatacenterCharacteristics(
                arch, os, vmm, hostList, time_zone, cost, costPerMem, costPerStorage, costPerBw);

        // Use CustomDatacenter instead of Datacenter
        Datacenter datacenter = null;
        try {
            datacenter = new CustomDatacenter(
                    "Datacenter_0",
                    characteristics,
                    new VmAllocationPolicySimple(hostList), // Pass hostList here
                    storageList,
                    0
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        return datacenter;
    }

    private static List<Host> CreateHosts() {
        List<Host> hostList = new ArrayList<>();

        int hostMips = 10000;        // Processing power of the host
        int hostRam = 16384;         // Host memory in MB
        long hostStorage = 1_000_000; // Host storage in MB
        long hostBw = 10_000;        // Bandwidth in MBps

        for (int i = 0; i < 4; i++) { // Create 4 hosts
            List<Pe> peList = new ArrayList<>();
            peList.add(new Pe(0, new PeProvisionerSimple(hostMips))); // One processing element (core)

            // Create a NetworkHost with all required arguments
            Host host = new NetworkHost(
                    i,                                    // Unique ID for the host
                    new RamProvisionerSimple(hostRam),    // RAM provisioner
                    new BwProvisionerSimple(hostBw),      // Bandwidth provisioner
                    hostStorage,                          // Storage capacity
                    peList,                               // List of processing elements (cores)
                    new VmSchedulerTimeShared(peList)     // VM Scheduler
            );

            hostList.add(host);
        }

        return hostList;
    }
}
