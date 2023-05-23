#pragma once

namespace {{namespace}} {

/// Sets the AMENT_PREFIX_PATH to the install space of this package.
///
/// This function needs to be called prior to any attempt to use a plugin in this package.
///
/// @param allow_appending If true, the install space is appended to the existing AMENT_PREFIX_PATH.
/// @note Using append=True can lead to unexpected behavior when another version of a
/// plugin used in this package is already present in the AMENT_PREFIX_PATH.
void SetUpAmentPrefixPath(bool allow_appending = false);

} // namespace {{namespace}}
