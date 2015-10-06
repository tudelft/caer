#include "davis_common.h"

static void createDefaultConfiguration(caerModuleData moduleData, struct caer_davis_info *devInfo);
static void sendDefaultConfiguration(caerModuleData moduleData, struct caer_davis_info *devInfo);
static void mainloopDataNotifyIncrease(void *p);
static void mainloopDataNotifyDecrease(void *p);
static void moduleShutdownNotify(void *p);
static void biasConfigSend(sshsNode node, caerModuleData moduleData);
static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void chipConfigSend(sshsNode node, caerModuleData moduleData);
static void chipConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void muxConfigSend(sshsNode node, caerModuleData moduleData);
static void muxConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void dvsConfigSend(sshsNode node, caerModuleData moduleData, struct caer_davis_info *devInfo);
static void dvsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void apsConfigSend(sshsNode node, caerModuleData moduleData, struct caer_davis_info *devInfo);
static void apsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void imuConfigSend(sshsNode node, caerModuleData moduleData);
static void imuConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void extInputConfigSend(sshsNode node, caerModuleData moduleData, struct caer_davis_info *devInfo);
static void extInputConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void usbConfigSend(sshsNode node, caerModuleData moduleData);
static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void systemConfigSend(sshsNode node, caerModuleData moduleData);
static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void createVDACBiasSetting(sshsNode biasNode, const char *biasName, uint8_t currentValue, uint8_t voltageValue);
static uint16_t generateVDACBias(sshsNode biasNode, const char *biasName);
static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type, const char *sex,
	uint8_t coarseValue, uint8_t fineValue, bool enabled);
static uint16_t generateCoarseFineBias(sshsNode biasNode, const char *biasName);
static void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue, uint8_t refValue,
	const char *operatingMode, const char *voltageLevel);
static uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName);

bool caerInputDAVISInit(caerModuleData moduleData, uint16_t deviceType) {
	caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

	// USB port/bus/SN settings/restrictions.
	// These can be used to force connection to one specific device at startup.
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "BusNumber", 0);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "DevAddress", 0);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "SerialNumber", "");

	// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "Auto-Restart", true);

	/// Start data acquisition, and correctly notify mainloop of new data and module of exceptional
	// shutdown cases (device pulled, ...).
	char *serialNumber = sshsNodeGetString(moduleData->moduleNode, "SerialNumber");
	moduleData->moduleState = caerDeviceOpen(moduleData->moduleID, deviceType,
		sshsNodeGetByte(moduleData->moduleNode, "BusNumber"), sshsNodeGetByte(moduleData->moduleNode, "DevAddress"),
		serialNumber);
	free(serialNumber);

	if (moduleData->moduleState == NULL) {
		// Failed to open device.
		return (false);
	}

	// Put global source information into SSHS.
	struct caer_davis_info devInfo = caerDavisInfoGet(moduleData->moduleState);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");

	sshsNodePutShort(sourceInfoNode, "logicVersion", devInfo.logicVersion);
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster", devInfo.deviceIsMaster);
	sshsNodePutShort(sourceInfoNode, "chipID", devInfo.chipID);

	sshsNodePutShort(sourceInfoNode, "dvsSizeX", devInfo.dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", devInfo.dvsSizeY);
	sshsNodePutBool(sourceInfoNode, "dvsHasPixelFilter", devInfo.dvsHasPixelFilter);
	sshsNodePutBool(sourceInfoNode, "dvsHasBackgroundActivityFilter", devInfo.dvsHasBackgroundActivityFilter);
	sshsNodePutBool(sourceInfoNode, "dvsHasTestEventGenerator", devInfo.dvsHasTestEventGenerator);

	sshsNodePutShort(sourceInfoNode, "apsSizeX", devInfo.apsSizeX);
	sshsNodePutShort(sourceInfoNode, "apsSizeY", devInfo.apsSizeY);
	sshsNodePutByte(sourceInfoNode, "apsColorFilter", devInfo.apsColorFilter);
	sshsNodePutBool(sourceInfoNode, "apsHasGlobalShutter", devInfo.apsHasGlobalShutter);
	sshsNodePutBool(sourceInfoNode, "apsHasQuadROI", devInfo.apsHasQuadROI);
	sshsNodePutBool(sourceInfoNode, "apsHasExternalADC", devInfo.apsHasExternalADC);
	sshsNodePutBool(sourceInfoNode, "apsHasInternalADC", devInfo.apsHasInternalADC);

	sshsNodePutBool(sourceInfoNode, "extInputHasGenerator", devInfo.extInputHasGenerator);

	caerModuleSetSubSystemString(moduleData, devInfo.deviceString);

	// Ensure good defaults for data acquisition settings.
	// No blocking behavior due to mainloop notification, and no auto-start of
	// all producers to ensure cAER settings are respected.
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, false);
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS, false);
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS, true);

	// Create default settings and send them to the device.
	createDefaultConfiguration(moduleData, &devInfo);
	sendDefaultConfiguration(moduleData, &devInfo);

	// Start data acquisition.
	bool ret = caerDeviceDataStart(moduleData->moduleState, &mainloopDataNotifyIncrease, &mainloopDataNotifyDecrease,
	NULL, &moduleShutdownNotify, moduleData->moduleNode);

	if (!ret) {
		// Failed to start data acquisition, close device and exit.
		caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

		return (false);
	}

	return (true);
}

void caerInputDAVISExit(caerModuleData moduleData) {
	caerDeviceDataStop(moduleData->moduleState);

	caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

	if (sshsNodeGetBool(moduleData->moduleNode, "Auto-Restart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "shutdown", false);
	}
}

void caerInputDAVISRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer *container = va_arg(args, caerEventPacketContainer *);

	*container = caerDeviceDataGet(moduleData->moduleState);

	if (*container != NULL) {
		caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);
	}
}

static void createDefaultConfiguration(caerModuleData moduleData, struct caer_davis_info *devInfo) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNode biasNode = sshsGetRelativeNode(moduleData->moduleNode, "bias/");

	if (devInfo->chipID == DAVIS_CHIP_DAVIS240A || devInfo->chipID == DAVIS_CHIP_DAVIS240B
		|| devInfo->chipID == DAVIS_CHIP_DAVIS240C) {
		createCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 4, 0, true);
		createCoarseFineBiasSetting(biasNode, "ApsCasEpc", "Cascode", "N", 5, 185, true);
		createCoarseFineBiasSetting(biasNode, "DiffCasBnc", "Cascode", "N", 5, 115, true);
		createCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 7, 152, true);
		createCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biasNode, "ApsOverflowLevel", "Normal", "N", 6, 253, true);

		createCoarseFineBiasSetting(biasNode, "BiasBuffer", "Normal", "N", 5, 254, true);

		createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "ShiftedSource", "SplitGate");
		createShiftedSourceBiasSetting(biasNode, "SSN", 33, 1, "ShiftedSource", "SplitGate");
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVIS640) {
		// Slow down pixels for big 640x480 array.
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 2, 3, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 1, true);
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVIS128 || devInfo->chipID == DAVIS_CHIP_DAVIS346A
		|| devInfo->chipID == DAVIS_CHIP_DAVIS346B || devInfo->chipID == DAVIS_CHIP_DAVIS346C
		|| devInfo->chipID == DAVIS_CHIP_DAVIS640 || devInfo->chipID == DAVIS_CHIP_DAVIS208) {
		createVDACBiasSetting(biasNode, "ApsOverflowLevel", 6, 27);
		createVDACBiasSetting(biasNode, "ApsCas", 6, 21);
		createVDACBiasSetting(biasNode, "AdcRefHigh", 7, 30);
		createVDACBiasSetting(biasNode, "AdcRefLow", 7, 1);
		if (devInfo->chipID == DAVIS_CHIP_DAVIS346A || devInfo->chipID == DAVIS_CHIP_DAVIS346B
			|| devInfo->chipID == DAVIS_CHIP_DAVIS346C || devInfo->chipID == DAVIS_CHIP_DAVIS640) {
			// Only DAVIS346 and 640 have ADC testing.
			createVDACBiasSetting(biasNode, "AdcTestVoltage", 7, 21);
		}

		createCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 4, 1, true);
		createCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biasNode, "ReadoutBufBp", "Normal", "P", 6, 20, true);
		createCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biasNode, "AdcCompBp", "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biasNode, "ColSelLowBn", "Normal", "N", 0, 1, true);
		createCoarseFineBiasSetting(biasNode, "DACBufBp", "Normal", "P", 6, 60, true);
		createCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 7, 152, true);
		createCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 5, 255, true);

		createCoarseFineBiasSetting(biasNode, "BiasBuffer", "Normal", "N", 5, 254, true);

		createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "ShiftedSource", "SplitGate");
		createShiftedSourceBiasSetting(biasNode, "SSN", 33, 1, "ShiftedSource", "SplitGate");
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVIS208) {
		createVDACBiasSetting(biasNode, "ResetHighPass", 7, 63);
		createVDACBiasSetting(biasNode, "RefSS", 5, 11);

		createCoarseFineBiasSetting(biasNode, "RegBiasBp", "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biasNode, "RefSSBn", "Normal", "N", 5, 20, true);
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVISRGB) {
		createVDACBiasSetting(biasNode, "ApsCas", 4, 21);
		createVDACBiasSetting(biasNode, "OVG1Lo", 4, 21);
		createVDACBiasSetting(biasNode, "OVG2Lo", 0, 0);
		createVDACBiasSetting(biasNode, "TX2OVG2Hi", 0, 63);
		createVDACBiasSetting(biasNode, "Gnd07", 4, 13);
		createVDACBiasSetting(biasNode, "AdcTestVoltage", 0, 21);
		createVDACBiasSetting(biasNode, "AdcRefHigh", 7, 63);
		createVDACBiasSetting(biasNode, "AdcRefLow", 7, 0);

		createCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 5, 255, false);
		createCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 5, 255, false);
		createCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 5, 164, false);
		createCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 7, 209, false);
		createCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 4, 164, true);
		createCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 4, 54, true);
		createCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 6, 63, true);
		createCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 2, 138, true);
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 1, 108, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 108, true);
		createCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 4, 28, true);
		createCoarseFineBiasSetting(biasNode, "ArrayBiasBufferBn", "Normal", "N", 6, 128, true);
		createCoarseFineBiasSetting(biasNode, "ArrayLogicBufferBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "FalltimeBn", "Normal", "N", 7, 41, true);
		createCoarseFineBiasSetting(biasNode, "RisetimeBp", "Normal", "P", 6, 162, true);
		createCoarseFineBiasSetting(biasNode, "ReadoutBufBp", "Normal", "P", 6, 20, false);
		createCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 6, 255, true);
		createCoarseFineBiasSetting(biasNode, "AdcCompBp", "Normal", "P", 4, 159, true);
		createCoarseFineBiasSetting(biasNode, "DACBufBp", "Normal", "P", 6, 194, true);
		createCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 7, 152, true);

		createCoarseFineBiasSetting(biasNode, "BiasBuffer", "Normal", "N", 6, 251, true);

		createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "TiedToRail", "SplitGate");
		createShiftedSourceBiasSetting(biasNode, "SSN", 33, 2, "ShiftedSource", "SplitGate");
	}

	sshsNode chipNode = sshsGetRelativeNode(moduleData->moduleNode, "chip/");

	sshsNodePutByteIfAbsent(chipNode, "DigitalMux0", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux1", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux2", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux3", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux0", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux1", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux2", 0);
	sshsNodePutByteIfAbsent(chipNode, "BiasMux0", 0);

	sshsNodePutBoolIfAbsent(chipNode, "ResetCalibNeuron", true);
	sshsNodePutBoolIfAbsent(chipNode, "TypeNCalibNeuron", false);
	sshsNodePutBoolIfAbsent(chipNode, "ResetTestPixel", true);
	sshsNodePutBoolIfAbsent(chipNode, "AERnArow", false); // Use nArow in the AER state machine.
	sshsNodePutBoolIfAbsent(chipNode, "UseAOut", false); // Enable analog pads for aMUX output (testing).

	if (devInfo->chipID == DAVIS_CHIP_DAVIS240A || devInfo->chipID == DAVIS_CHIP_DAVIS240B) {
		sshsNodePutBoolIfAbsent(chipNode, "SpecialPixelControl", false);
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVIS128 || devInfo->chipID == DAVIS_CHIP_DAVIS208
		|| devInfo->chipID == DAVIS_CHIP_DAVIS346A || devInfo->chipID == DAVIS_CHIP_DAVIS346B
		|| devInfo->chipID == DAVIS_CHIP_DAVIS346C || devInfo->chipID == DAVIS_CHIP_DAVIS640
		|| devInfo->chipID == DAVIS_CHIP_DAVISRGB) {
		// Select which grey counter to use with the internal ADC: '0' means the external grey counter is used, which
		// has to be supplied off-chip. '1' means the on-chip grey counter is used instead.
		sshsNodePutBoolIfAbsent(chipNode, "SelectGrayCounter", true);
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVIS346A || devInfo->chipID == DAVIS_CHIP_DAVIS346B
		|| devInfo->chipID == DAVIS_CHIP_DAVIS346C || devInfo->chipID == DAVIS_CHIP_DAVIS640
		|| devInfo->chipID == DAVIS_CHIP_DAVISRGB) {
		// Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the
		// VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.
		sshsNodePutBoolIfAbsent(chipNode, "TestADC", false);
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVIS208) {
		sshsNodePutBoolIfAbsent(chipNode, "SelectPreAmpAvg", false);
		sshsNodePutBoolIfAbsent(chipNode, "SelectBiasRefSS", false);
		sshsNodePutBoolIfAbsent(chipNode, "SelectSense", true);
		sshsNodePutBoolIfAbsent(chipNode, "SelectPosFb", false);
		sshsNodePutBoolIfAbsent(chipNode, "SelectHighPass", false);
	}

	if (devInfo->chipID == DAVIS_CHIP_DAVISRGB) {
		sshsNodePutBoolIfAbsent(chipNode, "AdjustOVG1Lo", true);
		sshsNodePutBoolIfAbsent(chipNode, "AdjustOVG2Lo", false);
		sshsNodePutBoolIfAbsent(chipNode, "AdjustTX2OVG2Hi", false);
	}

	// Subsystem 0: Multiplexer
	sshsNode muxNode = sshsGetRelativeNode(moduleData->moduleNode, "multiplexer/");

	sshsNodePutBoolIfAbsent(muxNode, "Run", true);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampRun", true);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampReset", false);
	sshsNodePutBoolIfAbsent(muxNode, "ForceChipBiasEnable", false);
	sshsNodePutBoolIfAbsent(muxNode, "DropDVSOnTransferStall", true);
	sshsNodePutBoolIfAbsent(muxNode, "DropAPSOnTransferStall", false);
	sshsNodePutBoolIfAbsent(muxNode, "DropIMUOnTransferStall", false);
	sshsNodePutBoolIfAbsent(muxNode, "DropExtInputOnTransferStall", true);

	sshsNodeAddAttrListener(muxNode, moduleData, &muxConfigListener);

	// Subsystem 1: DVS AER
	sshsNode dvsNode = sshsGetRelativeNode(moduleData->moduleNode, "dvs/");

	sshsNodePutBoolIfAbsent(dvsNode, "Run", true);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelayRow", 4);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelayColumn", 0);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtensionRow", 1);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtensionColumn", 0);
	sshsNodePutBoolIfAbsent(dvsNode, "WaitOnTransferStall", false);
	sshsNodePutBoolIfAbsent(dvsNode, "FilterRowOnlyEvents", true);
	sshsNodePutBoolIfAbsent(dvsNode, "ExternalAERControl", false);

	if (devInfo->dvsHasPixelFilter) {
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel0Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel0Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel1Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel1Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel2Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel2Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel3Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel3Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel4Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel4Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel5Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel5Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel6Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel6Column", devInfo->dvsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel7Row", devInfo->dvsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel7Column", devInfo->dvsSizeX);
	}

	if (devInfo->dvsHasBackgroundActivityFilter) {
		sshsNodePutBoolIfAbsent(dvsNode, "FilterBackgroundActivity", true);
		sshsNodePutIntIfAbsent(dvsNode, "FilterBackgroundActivityDeltaTime", 20000); // in µs
	}

	if (devInfo->dvsHasTestEventGenerator) {
		sshsNodePutBoolIfAbsent(dvsNode, "TestEventGeneratorEnable", false);
	}

	sshsNodeAddAttrListener(dvsNode, moduleData, &dvsConfigListener);

	// Subsystem 2: APS ADC
	sshsNode apsNode = sshsGetRelativeNode(moduleData->moduleNode, "aps/");

	// Only support GS on chips that have it available.
	if (devInfo->apsHasGlobalShutter) {
		sshsNodePutBoolIfAbsent(apsNode, "GlobalShutter", true);
	}

	sshsNodePutBoolIfAbsent(apsNode, "Run", true);
	sshsNodePutBoolIfAbsent(apsNode, "ResetRead", true);
	sshsNodePutBoolIfAbsent(apsNode, "WaitOnTransferStall", true);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn0", 0);
	sshsNodePutShortIfAbsent(apsNode, "StartRow0", 0);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn0", U16T(devInfo->apsSizeX - 1));
	sshsNodePutShortIfAbsent(apsNode, "EndRow0", U16T(devInfo->apsSizeY - 1));
	sshsNodePutIntIfAbsent(apsNode, "Exposure", 4000); // in µs
	sshsNodePutIntIfAbsent(apsNode, "FrameDelay", 1000); // in µs
	sshsNodePutShortIfAbsent(apsNode, "ResetSettle", 10); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "ColumnSettle", 30); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "RowSettle", 8); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "NullSettle", 3); // in cycles

	if (devInfo->apsHasQuadROI) {
		sshsNodePutShortIfAbsent(apsNode, "StartColumn1", devInfo->apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "StartRow1", devInfo->apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "EndColumn1", devInfo->apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "EndRow1", devInfo->apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "StartColumn2", devInfo->apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "StartRow2", devInfo->apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "EndColumn2", devInfo->apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "EndRow2", devInfo->apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "StartColumn3", devInfo->apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "StartRow3", devInfo->apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "EndColumn3", devInfo->apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "EndRow3", devInfo->apsSizeY);
	}

	if (devInfo->apsHasInternalADC) {
		sshsNodePutBoolIfAbsent(apsNode, "UseInternalADC", true);
		sshsNodePutBoolIfAbsent(apsNode, "SampleEnable", true);
		sshsNodePutShortIfAbsent(apsNode, "SampleSettle", 60); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "RampReset", 10); // in cycles
		sshsNodePutBoolIfAbsent(apsNode, "RampShortReset", true);
	}

	// DAVIS RGB has additional timing counters.
	if (devInfo->chipID == DAVIS_CHIP_DAVISRGB) {
		sshsNodePutShortIfAbsent(apsNode, "TransferTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "RSFDSettleTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSPDResetTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSResetFallTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSTXFallTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSFDResetTime", 3000); // in cycles
	}

	sshsNodeAddAttrListener(apsNode, moduleData, &apsConfigListener);

	// Subsystem 3: IMU
	sshsNode imuNode = sshsGetRelativeNode(moduleData->moduleNode, "imu/");

	sshsNodePutBoolIfAbsent(imuNode, "Run", true);
	sshsNodePutBoolIfAbsent(imuNode, "TempStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "AccelXStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "AccelYStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "AccelZStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "GyroXStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "GyroYStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "GyroZStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "LowPowerCycle", false);
	sshsNodePutByteIfAbsent(imuNode, "LowPowerWakeupFrequency", 1);
	sshsNodePutByteIfAbsent(imuNode, "SampleRateDivider", 0);
	sshsNodePutByteIfAbsent(imuNode, "DigitalLowPassFilter", 1);
	sshsNodePutByteIfAbsent(imuNode, "AccelFullScale", 1);
	sshsNodePutByteIfAbsent(imuNode, "GyroFullScale", 1);

	sshsNodeAddAttrListener(imuNode, moduleData, &imuConfigListener);

	// Subsystem 4: External Input
	sshsNode extNode = sshsGetRelativeNode(moduleData->moduleNode, "externalInput/");

	sshsNodePutBoolIfAbsent(extNode, "RunDetector", false);
	sshsNodePutBoolIfAbsent(extNode, "DetectRisingEdges", false);
	sshsNodePutBoolIfAbsent(extNode, "DetectFallingEdges", false);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulses", true);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulsePolarity", true);
	sshsNodePutIntIfAbsent(extNode, "DetectPulseLength", 10);

	if (devInfo->extInputHasGenerator) {
		sshsNodePutBoolIfAbsent(extNode, "RunGenerator", false);
		sshsNodePutBoolIfAbsent(extNode, "GenerateUseCustomSignal", false);
		sshsNodePutBoolIfAbsent(extNode, "GeneratePulsePolarity", true);
		sshsNodePutIntIfAbsent(extNode, "GeneratePulseInterval", 10);
		sshsNodePutIntIfAbsent(extNode, "GeneratePulseLength", 5);
	}

	sshsNodeAddAttrListener(extNode, moduleData, &extInputConfigListener);

	// Subsystem 9: FX2/3 USB Configuration and USB buffer settings.
	sshsNode usbNode = sshsGetRelativeNode(moduleData->moduleNode, "usb/");
	sshsNodePutBoolIfAbsent(usbNode, "Run", true);
	sshsNodePutShortIfAbsent(usbNode, "EarlyPacketDelay", 8); // 125µs time-slices, so 1ms

	sshsNodePutIntIfAbsent(usbNode, "BufferNumber", 8);
	sshsNodePutIntIfAbsent(usbNode, "BufferSize", 8192);

	sshsNodeAddAttrListener(usbNode, moduleData, &usbConfigListener);

	sshsNode sysNode = sshsGetRelativeNode(moduleData->moduleNode, "system/");

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(sysNode, "PacketContainerMaxSize", 4096 + 128 + 4 + 8);
	sshsNodePutIntIfAbsent(sysNode, "PacketContainerMaxInterval", 5000);
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxInterval", 1000);
	sshsNodePutIntIfAbsent(sysNode, "FramePacketMaxSize", 4);
	sshsNodePutIntIfAbsent(sysNode, "FramePacketMaxInterval", 50000);
	sshsNodePutIntIfAbsent(sysNode, "IMU6PacketMaxSize", 8);
	sshsNodePutIntIfAbsent(sysNode, "IMU6PacketMaxInterval", 5000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(sysNode, "DataExchangeBufferSize", 64);

	sshsNodeAddAttrListener(sysNode, moduleData, &systemConfigListener);
}

static void sendDefaultConfiguration(caerModuleData moduleData, struct caer_davis_info *devInfo) {
	// Send cAER configuration to libcaer and device.
	biasConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "bias/"), moduleData);
	chipConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "chip/"), moduleData);
	systemConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "system/"), moduleData);
	usbConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "usb/"), moduleData);
	muxConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "multiplexer/"), moduleData);
	dvsConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "dvs/"), moduleData, devInfo);
	apsConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "aps/"), moduleData, devInfo);
	imuConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "imu/"), moduleData);
	extInputConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "externalInput/"), moduleData, devInfo);
}

static void mainloopDataNotifyIncrease(void *p) {
	UNUSED_ARGUMENT(p);

	caerMainloopDataAvailableIncrease();
}

static void mainloopDataNotifyDecrease(void *p) {
	UNUSED_ARGUMENT(p);

	caerMainloopDataAvailableDecrease();
}

static void moduleShutdownNotify(void *p) {
	sshsNode moduleNode = p;

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(moduleNode, "shutdown", true);
}

static void biasConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void chipConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void chipConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void muxConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE,
		sshsNodeGetBool(node, "ForceChipBiasEnable"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL,
		sshsNodeGetBool(node, "DropDVSOnTransferStall"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL,
		sshsNodeGetBool(node, "DropAPSOnTransferStall"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL,
		sshsNodeGetBool(node, "DropIMUOnTransferStall"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL,
		sshsNodeGetBool(node, "DropExtInputOnTransferStall"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN,
		sshsNodeGetBool(node, "TimestampRun"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, sshsNodeGetBool(node, "Run"));
}

static void muxConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && caerStrEquals(changeKey, "TimestampReset")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "ForceChipBiasEnable")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DropDVSOnTransferStall")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DropAPSOnTransferStall")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DropIMUOnTransferStall")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DropExtInputOnTransferStall")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX,
				DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "TimestampRun")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "Run")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, changeValue.boolean);
		}
	}
}

static void dvsConfigSend(sshsNode node, caerModuleData moduleData, struct caer_davis_info *devInfo) {
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_ROW,
		sshsNodeGetByte(node, "AckDelayRow"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN,
		sshsNodeGetByte(node, "AckDelayColumn"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW,
		sshsNodeGetByte(node, "AckExtensionRow"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN,
		sshsNodeGetByte(node, "AckExtensionColumn"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL,
		sshsNodeGetBool(node, "WaitOnTransferStall"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS,
		sshsNodeGetBool(node, "FilterRowOnlyEvents"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL,
		sshsNodeGetBool(node, "ExternalAERControl"));

	if (devInfo->dvsHasPixelFilter) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW,
			sshsNodeGetShort(node, "FilterPixel0Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN,
			sshsNodeGetShort(node, "FilterPixel0Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW,
			sshsNodeGetShort(node, "FilterPixel1Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN,
			sshsNodeGetShort(node, "FilterPixel1Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW,
			sshsNodeGetShort(node, "FilterPixel2Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN,
			sshsNodeGetShort(node, "FilterPixel2Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW,
			sshsNodeGetShort(node, "FilterPixel3Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN,
			sshsNodeGetShort(node, "FilterPixel3Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW,
			sshsNodeGetShort(node, "FilterPixel4Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN,
			sshsNodeGetShort(node, "FilterPixel4Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW,
			sshsNodeGetShort(node, "FilterPixel5Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN,
			sshsNodeGetShort(node, "FilterPixel5Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW,
			sshsNodeGetShort(node, "FilterPixel6Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN,
			sshsNodeGetShort(node, "FilterPixel6Column"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW,
			sshsNodeGetShort(node, "FilterPixel7Row"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN,
			sshsNodeGetShort(node, "FilterPixel7Column"));
	}

	if (devInfo->dvsHasBackgroundActivityFilter) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY,
			sshsNodeGetBool(node, "FilterBackgroundActivity"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS,
		DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_DELTAT, sshsNodeGetInt(node, "FilterBackgroundActivityDeltaTime"));
	}

	if (devInfo->dvsHasTestEventGenerator) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE,
			sshsNodeGetBool(node, "TestEventGeneratorEnable"));
	}

	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, sshsNodeGetBool(node, "Run"));
}

static void dvsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BYTE && caerStrEquals(changeKey, "AckDelayRow")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_ROW,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "AckDelayColumn")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "AckExtensionRow")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "AckExtensionColumn")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN,
				changeValue.ubyte);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "WaitOnTransferStall")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "FilterRowOnlyEvents")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "ExternalAERControl")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL,
				changeValue.boolean);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel0Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel0Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel1Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel1Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel2Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel2Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel3Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel3Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel4Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel4Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel5Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel5Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel6Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel6Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel7Row")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "FilterPixel7Column")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN,
				changeValue.ushort);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "FilterBackgroundActivity")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY,
				changeValue.boolean);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "FilterBackgroundActivityDeltaTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS,
			DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_DELTAT, changeValue.uint);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "TestEventGeneratorEnable")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "Run")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, changeValue.boolean);
		}
	}
}

static void apsConfigSend(sshsNode node, caerModuleData moduleData, struct caer_davis_info *devInfo) {
	if (devInfo->apsHasGlobalShutter) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER,
			sshsNodeGetBool(node, "GlobalShutter"));
	}

	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ,
		sshsNodeGetBool(node, "ResetRead"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL,
		sshsNodeGetBool(node, "WaitOnTransferStall"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0,
		sshsNodeGetShort(node, "StartColumn0"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0,
		sshsNodeGetShort(node, "StartRow0"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0,
		sshsNodeGetShort(node, "EndColumn0"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0,
		sshsNodeGetShort(node, "EndRow0"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE,
		sshsNodeGetInt(node, "Exposure"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_DELAY,
		sshsNodeGetInt(node, "FrameDelay"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_SETTLE,
		sshsNodeGetShort(node, "ResetSettle"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_COLUMN_SETTLE,
		sshsNodeGetShort(node, "ColumnSettle"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROW_SETTLE,
		sshsNodeGetShort(node, "RowSettle"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_NULL_SETTLE,
		sshsNodeGetShort(node, "NullSettle"));

	if (devInfo->apsHasQuadROI) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_1,
			sshsNodeGetShort(node, "StartColumn1"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_1,
			sshsNodeGetShort(node, "StartRow1"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_1,
			sshsNodeGetShort(node, "EndColumn1"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_1,
			sshsNodeGetShort(node, "EndRow1"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_2,
			sshsNodeGetShort(node, "StartColumn2"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_2,
			sshsNodeGetShort(node, "StartRow2"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_2,
			sshsNodeGetShort(node, "EndColumn2"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_2,
			sshsNodeGetShort(node, "EndRow2"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_3,
			sshsNodeGetShort(node, "StartColumn3"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_3,
			sshsNodeGetShort(node, "StartRow3"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_3,
			sshsNodeGetShort(node, "EndColumn3"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_3,
			sshsNodeGetShort(node, "EndRow3"));
	}

	if (devInfo->apsHasInternalADC) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_USE_INTERNAL_ADC,
			sshsNodeGetBool(node, "UseInternalADC"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_ENABLE,
			sshsNodeGetBool(node, "SampleEnable"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_SETTLE,
			sshsNodeGetShort(node, "SampleSettle"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_RESET,
			sshsNodeGetShort(node, "RampReset"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_SHORT_RESET,
			sshsNodeGetBool(node, "RampShortReset"));
	}

	// DAVIS RGB extra timing support.
	if (devInfo->chipID == DAVIS_CHIP_DAVISRGB) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_TRANSFER,
			sshsNodeGetShort(node, "TransferTime"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_RSFDSETTLE,
			sshsNodeGetShort(node, "RSFDSettleTime"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSPDRESET,
			sshsNodeGetShort(node, "GSPDResetTime"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSRESETFALL,
			sshsNodeGetShort(node, "GSResetFallTime"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSTXFALL,
			sshsNodeGetShort(node, "GSTXFallTime"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSFDRESET,
			sshsNodeGetShort(node, "GSFDResetTime"));
	}

	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, sshsNodeGetBool(node, "Run"));
}

static void apsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && caerStrEquals(changeKey, "GlobalShutter")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "ResetRead")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "WaitOnTransferStall")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL,
				changeValue.boolean);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartColumn0")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartRow0")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndColumn0")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndRow0")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0,
				changeValue.ushort);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "Exposure")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "FrameDelay")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_DELAY,
				changeValue.uint);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "ResetSettle")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_SETTLE,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "ColumnSettle")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_COLUMN_SETTLE,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "RowSettle")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROW_SETTLE,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "NullSettle")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_NULL_SETTLE,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartColumn1")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_1,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartRow1")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_1,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndColumn1")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_1,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndRow1")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_1,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartColumn2")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_2,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartRow2")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_2,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndColumn2")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_2,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndRow2")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_2,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartColumn3")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_3,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "StartRow3")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_3,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndColumn3")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_3,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EndRow3")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_3,
				changeValue.ushort);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "UseInternalADC")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_USE_INTERNAL_ADC,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "SampleEnable")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_ENABLE,
				changeValue.boolean);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "SampleSettle")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_SETTLE,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "RampReset")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_RESET,
				changeValue.ushort);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "RampShortReset")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_SHORT_RESET,
				changeValue.boolean);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "TransferTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_TRANSFER,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "RSFDSettleTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_RSFDSETTLE,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "GSPDResetTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSPDRESET,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "GSResetFallTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSRESETFALL,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "GSTXFallTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSTXFALL,
				changeValue.ushort);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "GSFDResetTime")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSFDRESET,
				changeValue.ushort);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "Run")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, changeValue.boolean);
		}
	}
}

static void imuConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_TEMP_STANDBY,
		sshsNodeGetBool(node, "TempStandby"));

	uint8_t accelStandby = 0;
	accelStandby |= U8T(sshsNodeGetBool(node, "AccelXStandby") << 2);
	accelStandby |= U8T(sshsNodeGetBool(node, "AccelYStandby") << 1);
	accelStandby |= U8T(sshsNodeGetBool(node, "AccelZStandby") << 0);
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_STANDBY, accelStandby);

	uint8_t gyroStandby = 0;
	gyroStandby |= U8T(sshsNodeGetBool(node, "GyroXStandby") << 2);
	gyroStandby |= U8T(sshsNodeGetBool(node, "GyroYStandby") << 1);
	gyroStandby |= U8T(sshsNodeGetBool(node, "GyroZStandby") << 0);
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_STANDBY, gyroStandby);

	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_CYCLE,
		sshsNodeGetBool(node, "LowPowerCycle"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_WAKEUP,
		sshsNodeGetByte(node, "LowPowerWakeupFrequency"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER,
		sshsNodeGetByte(node, "SampleRateDivider"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER,
		sshsNodeGetByte(node, "DigitalLowPassFilter"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE,
		sshsNodeGetByte(node, "AccelFullScale"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE,
		sshsNodeGetByte(node, "GyroFullScale"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, sshsNodeGetBool(node, "Run"));
}

static void imuConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && caerStrEquals(changeKey, "TempStandby")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_TEMP_STANDBY,
				changeValue.boolean);
		}
		else if (changeType == BOOL
			&& (caerStrEquals(changeKey, "AccelXStandby") || caerStrEquals(changeKey, "AccelYStandby")
				|| caerStrEquals(changeKey, "AccelZStandby"))) {
			uint8_t accelStandby = 0;
			accelStandby |= U8T(sshsNodeGetBool(node, "AccelXStandby") << 2);
			accelStandby |= U8T(sshsNodeGetBool(node, "AccelYStandby") << 1);
			accelStandby |= U8T(sshsNodeGetBool(node, "AccelZStandby") << 0);

			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_STANDBY,
				accelStandby);
		}
		else if (changeType == BOOL
			&& (caerStrEquals(changeKey, "GyroXStandby") || caerStrEquals(changeKey, "GyroYStandby")
				|| caerStrEquals(changeKey, "GyroZStandby"))) {
			uint8_t gyroStandby = 0;
			gyroStandby |= U8T(sshsNodeGetBool(node, "GyroXStandby") << 2);
			gyroStandby |= U8T(sshsNodeGetBool(node, "GyroYStandby") << 1);
			gyroStandby |= U8T(sshsNodeGetBool(node, "GyroZStandby") << 0);

			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_STANDBY, gyroStandby);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "LowPowerCycle")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_CYCLE,
				changeValue.boolean);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "LowPowerWakeupFrequency")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_WAKEUP,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "SampleRateDivider")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "DigitalLowPassFilter")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "AccelFullScale")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE,
				changeValue.ubyte);
		}
		else if (changeType == BYTE && caerStrEquals(changeKey, "GyroFullScale")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE,
				changeValue.ubyte);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "Run")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, changeValue.boolean);
		}
	}
}

static void extInputConfigSend(sshsNode node, caerModuleData moduleData, struct caer_davis_info *devInfo) {
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES,
		sshsNodeGetBool(node, "DetectRisingEdges"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES,
		sshsNodeGetBool(node, "DetectFallingEdges"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES,
		sshsNodeGetBool(node, "DetectPulses"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY,
		sshsNodeGetBool(node, "DetectPulsePolarity"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH,
		sshsNodeGetInt(node, "DetectPulseLength"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR,
		sshsNodeGetBool(node, "RunDetector"));

	if (devInfo->extInputHasGenerator) {
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
		DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL, sshsNodeGetBool(node, "GenerateUseCustomSignal"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
		DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, sshsNodeGetBool(node, "GeneratePulsePolarity"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
		DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL, sshsNodeGetInt(node, "GeneratePulseInterval"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
			sshsNodeGetInt(node, "GeneratePulseLength"));
		caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR,
			sshsNodeGetBool(node, "RunGenerator"));
	}
}

static void extInputConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && caerStrEquals(changeKey, "DetectRisingEdges")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DetectFallingEdges")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DetectPulses")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "DetectPulsePolarity")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY, changeValue.boolean);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "DetectPulseLength")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH, changeValue.uint);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "RunDetector")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR,
				changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "GenerateUseCustomSignal")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL, changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "GeneratePulsePolarity")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, changeValue.boolean);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "GeneratePulseInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "GeneratePulseLength")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT,
			DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH, changeValue.uint);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "RunGenerator")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR,
				changeValue.boolean);
		}
	}
}

static void usbConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER,
		sshsNodeGetInt(node, "BufferNumber"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE,
		sshsNodeGetInt(node, "BufferSize"));

	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY,
		sshsNodeGetShort(node, "EarlyPacketDelay"));
	caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, sshsNodeGetBool(node, "Run"));
}

static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == INT && caerStrEquals(changeKey, "BufferNumber")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER,
				changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "BufferSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE,
				changeValue.uint);
		}
		else if (changeType == SHORT && caerStrEquals(changeKey, "EarlyPacketDelay")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY,
				changeValue.ushort);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "Run")) {
			caerDeviceConfigSet(moduleData->moduleState, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, changeValue.boolean);
		}
	}
}

static void systemConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_SIZE,
		sshsNodeGetInt(node, "PacketContainerMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, sshsNodeGetInt(node, "PacketContainerMaxInterval"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_SIZE,
		sshsNodeGetInt(node, "PolarityPacketMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_INTERVAL, sshsNodeGetInt(node, "PolarityPacketMaxInterval"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_SIZE,
		sshsNodeGetInt(node, "SpecialPacketMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_INTERVAL, sshsNodeGetInt(node, "SpecialPacketMaxInterval"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_FRAME_SIZE,
		sshsNodeGetInt(node, "FramePacketMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_FRAME_INTERVAL, sshsNodeGetInt(node, "FramePacketMaxInterval"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_IMU6_SIZE,
		sshsNodeGetInt(node, "IMU6PacketMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_IMU6_INTERVAL, sshsNodeGetInt(node, "IMU6PacketMaxInterval"));

	// Changes only take effect on module start!
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE, sshsNodeGetInt(node, "DataExchangeBufferSize"));
}

static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == INT && caerStrEquals(changeKey, "PacketContainerMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "PacketContainerMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "PolarityPacketMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "PolarityPacketMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "SpecialPacketMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "SpecialPacketMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "FramePacketMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_FRAME_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "FramePacketMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_FRAME_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "IMU6PacketMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_IMU6_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "IMU6PacketMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_IMU6_INTERVAL, changeValue.uint);
		}
	}
}

static void createVDACBiasSetting(sshsNode biasNode, const char *biasName, uint8_t currentValue, uint8_t voltageValue) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutByteIfAbsent(biasConfigNode, "currentValue", currentValue);
	sshsNodePutByteIfAbsent(biasConfigNode, "voltageValue", voltageValue);
}

static uint16_t generateVDACBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Build up bias value from all its components.
	struct caer_bias_vdac biasValue = { .voltageValue = sshsNodeGetByte(biasConfigNode, "voltageValue"), .currentValue =
		sshsNodeGetByte(biasConfigNode, "currentValue"), };

	return (caerBiasVDACGenerate(biasValue));
}

static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type, const char *sex,
	uint8_t coarseValue, uint8_t fineValue, bool enabled) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutStringIfAbsent(biasConfigNode, "type", type);
	sshsNodePutStringIfAbsent(biasConfigNode, "sex", sex);
	sshsNodePutByteIfAbsent(biasConfigNode, "coarseValue", coarseValue);
	sshsNodePutByteIfAbsent(biasConfigNode, "fineValue", fineValue);
	sshsNodePutBoolIfAbsent(biasConfigNode, "enabled", enabled);
	sshsNodePutStringIfAbsent(biasConfigNode, "currentLevel", "Normal");
}

static uint16_t generateCoarseFineBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Build up bias value from all its components.
	struct caer_bias_coarsefine biasValue = { .coarseValue = sshsNodeGetByte(biasConfigNode, "coarseValue"),
		.fineValue = sshsNodeGetByte(biasConfigNode, "fineValue"),
		.enabled = sshsNodeGetBool(biasConfigNode, "enabled"), .sexN = caerStrEquals(
			sshsNodeGetString(biasConfigNode, "sex"), "N"), .typeNormal = caerStrEquals(
			sshsNodeGetString(biasConfigNode, "type"), "Normal"), .currentLevelNormal = caerStrEquals(
			sshsNodeGetString(biasConfigNode, "currentLevel"), "Normal"), };

	return (caerBiasCoarseFineGenerate(biasValue));
}

static void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue, uint8_t refValue,
	const char *operatingMode, const char *voltageLevel) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutByteIfAbsent(biasConfigNode, "regValue", regValue);
	sshsNodePutByteIfAbsent(biasConfigNode, "refValue", refValue);
	sshsNodePutStringIfAbsent(biasConfigNode, "operatingMode", operatingMode);
	sshsNodePutStringIfAbsent(biasConfigNode, "voltageLevel", voltageLevel);
}

static uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Build up bias value from all its components.
	struct caer_bias_shiftedsource biasValue = { .refValue = sshsNodeGetByte(biasConfigNode, "refValue"), .regValue =
		sshsNodeGetByte(biasConfigNode, "regValue"), .operatingMode =
		(caerStrEquals(sshsNodeGetString(biasConfigNode, "operatingMode"), "HiZ")) ?
			(HI_Z) :
			((caerStrEquals(sshsNodeGetString(biasConfigNode, "operatingMode"), "TiedToRail")) ?
				(TIED_TO_RAIL) : (SHIFTED_SOURCE)), .voltageLevel =
		(caerStrEquals(sshsNodeGetString(biasConfigNode, "voltageLevel"), "SingleDiode")) ?
			(SINGLE_DIODE) :
			((caerStrEquals(sshsNodeGetString(biasConfigNode, "voltageLevel"), "DoubleDiode")) ?
				(DOUBLE_DIODE) : (SPLIT_GATE)), };

	return (caerBiasShiftedSourceGenerate(biasValue));
}
