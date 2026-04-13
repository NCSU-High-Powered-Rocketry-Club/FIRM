// Re-export the main class and types from your wrapper
export { FIRMClient as FIRM, type FIRMConnectOptions } from './FIRM.js';

// Re-export all the shared types from your types file
export {
  type FIRMPacket,
  type FIRMResponse,
  type DeviceInfo,
  type DeviceConfig,
  type DeviceProtocol,
  type CalibrationValues,
} from './types.js';
