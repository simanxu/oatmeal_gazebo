#ifndef RBDL_CONFIG_H_
#define RBDL_CONFIG_H_

#define RBDL_API_VERSION (3 << 16) + (0 << 8) + 0
#define RBDL_BUILD_COMMIT "unknown"
#define RBDL_BUILD_TYPE "unknown"
#define RBDL_BUILD_BRANCH "unknown"

// Disables these macro for static build.
#define RBDL_DLLAPI
#define RBDL_LOCAL
#define RBDL_ADDON_DLLAPI

#endif  // RBDL_CONFIG_H_
