import { ExtensionContext } from "@foxglove/extension";

import { initMonitorPanel } from "./MonitorPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "KAIEV26 Decision Monitor",
    initPanel: initMonitorPanel,
  });
}
