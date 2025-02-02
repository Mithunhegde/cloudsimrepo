package org.cloudbus.cloudsim.examples.storage;

import org.cloudbus.cloudsim.Datacenter;
import org.cloudbus.cloudsim.DatacenterCharacteristics;
import org.cloudbus.cloudsim.VmAllocationPolicy;
import org.cloudbus.cloudsim.Storage;

import java.util.List;

public class CustomDatacenter extends Datacenter {

    public CustomDatacenter(String name,
                            DatacenterCharacteristics characteristics,
                            VmAllocationPolicy vmAllocationPolicy,
                            List<Storage> storageList,
                            double schedulingInterval) throws Exception {
        super(name, characteristics, vmAllocationPolicy, storageList, schedulingInterval);
    }

    // Expose the protected getStorageList method as public
    @Override
    public List<Storage> getStorageList() {
        return super.getStorageList();
    }
}
