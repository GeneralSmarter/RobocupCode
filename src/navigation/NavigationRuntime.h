#ifndef NAVIGATION_RUNTIME_H
#define NAVIGATION_RUNTIME_H

// System scheduler boundary. Mission code must not include this header.
void initializeNavigationRuntime();
void updateNavigationRuntime();

#endif
