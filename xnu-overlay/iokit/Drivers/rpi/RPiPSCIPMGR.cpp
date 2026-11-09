/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>

#include "RPiPlatform.h"

extern "C" {
#include <machine/machine_routines.h>
}

extern "C" char LowResetVectorBase[];

#undef super
#define super IOPMGR
OSDefineMetaClassAndStructors(RPiPSCIPMGR, IOPMGR);

bool
RPiPSCIPMGR::start(IOService *provider)
{
	if (!super::start(provider) || !rpi_psci_init()) {
		return false;
	}

	IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
	if (chosen) {
		OSData *entry = OSDynamicCast(OSData, chosen->getProperty("pieboot,secondary-entry"));
		if (entry && entry->getLength() == sizeof(uint64_t)) {
			secondaryEntry = *(const uint64_t *)entry->getBytesNoCopy();
		}
		chosen->release();
	}

	registerService();
	return true;
}

void
RPiPSCIPMGR::enableCPUCore(unsigned int cpu_id, uint64_t entry_pa)
{
	const ml_topology_info *topology = ml_get_topology_info();
	uint64_t mpidr = topology->cpus[cpu_id].phys_id;

	if (entry_pa == 0) {
		entry_pa = ml_vtophys((vm_offset_t)LowResetVectorBase);
	}

	int64_t ret;
	if (secondaryEntry) {
		ret = rpi_psci_call(PSCI_0_2_FN64_CPU_ON, mpidr, secondaryEntry, entry_pa);
	} else {
		ret = rpi_psci_call(PSCI_0_2_FN64_CPU_ON, mpidr, entry_pa, 0);
	}
	if (ret != PSCI_RET_SUCCESS && ret != PSCI_RET_ALREADY_ON) {
		panic("RPiPSCIPMGR: CPU_ON(cpu %u, mpidr 0x%llx) failed: %lld", cpu_id, mpidr, ret);
	}
}

void
RPiPSCIPMGR::disableCPUCore(unsigned int  )
{
}

void
RPiPSCIPMGR::enableCPUCluster(unsigned int  )
{
}

void
RPiPSCIPMGR::disableCPUCluster(unsigned int  )
{
}

void
RPiPSCIPMGR::initCPUIdle(ml_processor_info_t *  )
{
}

void
RPiPSCIPMGR::enterCPUIdle(UInt64 *  )
{
}

void
RPiPSCIPMGR::exitCPUIdle(UInt64 *  )
{
}

void
RPiPSCIPMGR::updateCPUIdle(UInt64 *  )
{
}
