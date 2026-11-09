/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <kern/startup.h>
#include <libkern/libkern.h>
#include <libkern/amfi/amfi.h>

#pragma mark Entitlements

static void
builtin_OSEntitlements_invalidate(void *osentitlements __unused)
{
}

static void *
builtin_OSEntitlements_asdict(void *osentitlements __unused)
{
	return NULL;
}

static const char builtin_ce_not_found;

static CEError_t
builtin_OSEntitlements_query(void *osentitlements __unused, uint8_t cdhash[CS_CDHASH_LEN] __unused,
    CEQuery_t query __unused, size_t queryLength __unused)
{
	return (CEError_t)(const void *)&builtin_ce_not_found;
}

static bool
builtin_OSEntitlements_get_transmuted(void *osentitlements __unused, const CS_GenericBlob **blob)
{
	*blob = NULL;
	return false;
}

static bool
builtin_OSEntitlements_get_xml(void *osentitlements __unused, CS_GenericBlob **blob)
{
	*blob = NULL;
	return false;
}

static bool
builtin_get_legacy_profile_exemptions(const uint8_t **profile, size_t *profileLength)
{
	*profile = NULL;
	*profileLength = 0;
	return false;
}

static bool
builtin_get_udid(const uint8_t **udid, size_t *udidLength)
{
	*udid = NULL;
	*udidLength = 0;
	return false;
}

static void *
builtin_query_context_to_object(CEQueryContext_t ctx __unused)
{
	return NULL;
}

static kern_return_t
builtin_adjustContext(void *os_entitlements __unused, struct cs_blob *code_signing_blob __unused,
    const CEContext_t *ce_ctx __unused)
{
	return KERN_SUCCESS;
}

static kern_return_t
builtin_adjustContextWithMonitor(void *os_entitlements __unused, const CEQueryContext_t ce_ctx __unused,
    const void *monitor_sig_obj __unused, const char *identity __unused,
    const uint32_t code_signing_flags __unused)
{
	return KERN_SUCCESS;
}

static kern_return_t
builtin_adjustContextWithoutMonitor(void *os_entitlements __unused, struct cs_blob *code_signing_blob __unused)
{
	return KERN_SUCCESS;
}

static kern_return_t
builtin_queryEntitlementBoolean(const void *os_entitlements __unused, const char *entitlement_name __unused)
{
	return KERN_NOT_FOUND;
}

static kern_return_t
builtin_queryEntitlementBooleanWithProc(const proc_t proc __unused, const char *entitlement_name __unused)
{
	return KERN_NOT_FOUND;
}

static kern_return_t
builtin_queryEntitlementString(const void *os_entitlements __unused, const char *entitlement_name __unused,
    const char *entitlement_value __unused)
{
	return KERN_NOT_FOUND;
}

static kern_return_t
builtin_queryEntitlementStringWithProc(const proc_t proc __unused, const char *entitlement_name __unused,
    const char *entitlement_value __unused)
{
	return KERN_NOT_FOUND;
}

static kern_return_t
builtin_copyEntitlementAsOSObject(const void *os_entitlements __unused, const char *entitlement_name __unused,
    void **entitlement_object)
{
	*entitlement_object = NULL;
	return KERN_NOT_FOUND;
}

static kern_return_t
builtin_copyEntitlementAsOSObjectWithProc(const proc_t proc __unused, const char *entitlement_name __unused,
    void **entitlement_object)
{
	*entitlement_object = NULL;
	return KERN_NOT_FOUND;
}

#pragma mark Trust caches

static const TCReturn_t tc_not_found = { .error = kTCReturnNotFound };
static const TCReturn_t tc_unsupported = { .error = kTCReturnUnsupported };

static TCReturn_t
builtin_tc_loadModule(TrustCacheRuntime_t *runtime __unused, const TCType_t type __unused,
    TrustCache_t *trustCache __unused, const uintptr_t dataAddr __unused, const size_t dataSize __unused)
{
	return tc_unsupported;
}

static TCReturn_t
builtin_tc_load(TrustCacheRuntime_t *runtime __unused, TCType_t type __unused, TrustCache_t *trustCache __unused,
    const uintptr_t payloadAddr __unused, const size_t payloadSize __unused,
    const uintptr_t manifestAddr __unused, const size_t manifestSize __unused)
{
	return tc_unsupported;
}

static TCReturn_t
builtin_tc_query(const TrustCacheRuntime_t *runtime __unused, TCQueryType_t queryType __unused,
    const uint8_t CDHash[kTCEntryHashSize] __unused, TrustCacheQueryToken_t *queryToken __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_getCapabilities(const TrustCache_t *trustCache __unused, TCCapabilities_t *capabilities __unused)
{
	return tc_unsupported;
}

static TCReturn_t
builtin_tc_queryGetTCType(const TrustCacheQueryToken_t *queryToken __unused, TCType_t *typeRet __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_queryGetCapabilities(const TrustCacheQueryToken_t *queryToken __unused,
    TCCapabilities_t *capabilities __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_queryGetHashType(const TrustCacheQueryToken_t *queryToken __unused, uint8_t *hashTypeRet __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_queryGetFlags(const TrustCacheQueryToken_t *queryToken __unused, uint64_t *flagsRet __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_queryGetConstraintCategory(const TrustCacheQueryToken_t *queryToken __unused,
    uint8_t *constraintCategoryRet __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_queryGetUUID(const TrustCacheQueryToken_t *queryToken __unused, uint8_t returnUUID[kUUIDSize] __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_constructInvalid(TrustCache_t *trustCache __unused, const uint8_t *moduleAddr __unused,
    size_t moduleSize __unused)
{
	return tc_unsupported;
}

static TCReturn_t
builtin_tc_checkRuntimeForUUID(const TrustCacheRuntime_t *runtime __unused,
    const uint8_t checkUUID[kUUIDSize] __unused, const TrustCache_t **trustCacheRet __unused)
{
	return tc_not_found;
}

static TCReturn_t
builtin_tc_extractModule(TrustCache_t *trustCache __unused, const uint8_t *dataAddr __unused,
    size_t dataSize __unused)
{
	return tc_unsupported;
}

static TCReturn_t
builtin_tc_getModule(const TrustCache_t *trustCache __unused, const uint8_t **moduleAddrRet __unused,
    size_t *moduleSizeRet __unused)
{
	return tc_unsupported;
}

static TCReturn_t
builtin_tc_getUUID(const TrustCache_t *trustCache __unused, uint8_t returnUUID[kUUIDSize] __unused)
{
	return tc_unsupported;
}

#pragma mark MTE

static bool
builtin_has_mte_soft_mode(const proc_t proc __unused)
{
	return false;
}

static bool
builtin_mte_blob_false(struct cs_blob *blob __unused)
{
	return false;
}

#pragma mark Registration

static const amfi_t builtin_amfi = {
	.OSEntitlements_invalidate = builtin_OSEntitlements_invalidate,
	.OSEntitlements_asdict = builtin_OSEntitlements_asdict,
	.OSEntitlements_query = builtin_OSEntitlements_query,
	.OSEntitlements_get_transmuted = builtin_OSEntitlements_get_transmuted,
	.OSEntitlements_get_xml = builtin_OSEntitlements_get_xml,
	.get_legacy_profile_exemptions = builtin_get_legacy_profile_exemptions,
	.get_udid = builtin_get_udid,
	.query_context_to_object = builtin_query_context_to_object,
	.TrustCache = {
		.version = TRUST_CACHE_INTERFACE_VERSION,
		.loadModule = builtin_tc_loadModule,
		.load = builtin_tc_load,
		.query = builtin_tc_query,
		.getCapabilities = builtin_tc_getCapabilities,
		.queryGetTCType = builtin_tc_queryGetTCType,
		.queryGetCapabilities = builtin_tc_queryGetCapabilities,
		.queryGetHashType = builtin_tc_queryGetHashType,
		.queryGetFlags = builtin_tc_queryGetFlags,
		.queryGetConstraintCategory = builtin_tc_queryGetConstraintCategory,
		.queryGetUUID = builtin_tc_queryGetUUID,
		.constructInvalid = builtin_tc_constructInvalid,
		.checkRuntimeForUUID = builtin_tc_checkRuntimeForUUID,
		.extractModule = builtin_tc_extractModule,
		.getModule = builtin_tc_getModule,
		.getUUID = builtin_tc_getUUID,
	},
	.OSEntitlements = {
		.version = OSENTITLEMENTS_INTERFACE_VERSION,
		.adjustContext = builtin_adjustContext,
		.adjustContextWithMonitor = builtin_adjustContextWithMonitor,
		.adjustContextWithoutMonitor = builtin_adjustContextWithoutMonitor,
		.queryEntitlementBoolean = builtin_queryEntitlementBoolean,
		.queryEntitlementBooleanWithProc = builtin_queryEntitlementBooleanWithProc,
		.queryEntitlementString = builtin_queryEntitlementString,
		.queryEntitlementStringWithProc = builtin_queryEntitlementStringWithProc,
		.copyEntitlementAsOSObject = builtin_copyEntitlementAsOSObject,
		.copyEntitlementAsOSObjectWithProc = builtin_copyEntitlementAsOSObjectWithProc,
	},
	.has_mte_soft_mode = builtin_has_mte_soft_mode,
	.has_mte_opt_out = builtin_mte_blob_false,
	.has_mte_inheritance_opt_out = builtin_mte_blob_false,
	.has_mte_data_tagging_opt_out = builtin_mte_blob_false,
	.has_mte_alias_restriction_opt_in = builtin_mte_blob_false,
};

__startup_func
static void
builtin_amfi_init(void)
{
	amfi_interface_register(&builtin_amfi);
}
STARTUP(SYSCTL, STARTUP_RANK_LAST, builtin_amfi_init);
