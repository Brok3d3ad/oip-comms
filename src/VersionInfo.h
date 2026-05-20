#ifndef VERSIONINFO_H
#define VERSIONINFO_H

// Paho MQTT C normally generates this header via CMake. We build Paho's sources
// inline from the submodule at paho/, so we provide a static stub matching the
// pinned upstream tag. Update CLIENT_VERSION when bumping the submodule.

#define BUILD_TIMESTAMP "inline"
#define CLIENT_VERSION  "1.3.13"

#endif /* VERSIONINFO_H */
