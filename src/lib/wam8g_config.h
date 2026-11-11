#pragma once

/* Configuration checks for the wam8g programs (leader_wam8g,
 * dynamics_excitation_wam8g), as in the haptic demos' wam8g_support.h.
 * Nothing here opens the CAN bus. */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <libgen.h>
#include <unistd.h>

#include <libconfig.h++>
#include <barrett/products/product_manager.h>

namespace wam8g_config {

/* Wam8g refuses Cartesian control and gravity compensation until the
 * operator marks the profile calibrated. Both programs need gravity
 * compensation, so they check the flags before any puck is woken. */
inline bool flagSet(const libconfig::Setting& profile, const char* name) {
	bool value = false;
	return profile.lookupValue(name, value) && value;
}

inline bool configReady(const libconfig::Config& config, const char* program) {
	if (!config.exists("wam8g")) {
		printf("%s: the configuration has no wam8g profile.\n", program);
		return false;
	}
	const libconfig::Setting& profile = config.lookup("wam8g");
	bool ready = true;
	if (!flagSet(profile, "kinematics_calibrated")) {
		printf("%s: wam8g.conf does not set kinematics_calibrated = true.\n", program);
		ready = false;
	}
	if (!flagSet(profile, "gravity_calibrated")) {
		printf("%s: wam8g.conf does not set gravity_calibrated = true.\n", program);
		ready = false;
	}
	return ready;
}

/* Read a configuration file from its own directory, as ProductManager does,
 * so its @include paths resolve. (libconfig 1.4, installed beside the legacy
 * libbarrett, has no setIncludeDir().) */
inline bool readConfig(const char* file, libconfig::Config& config, const char* program) {
	std::string a(file), b(file);
	const std::string dir(dirname(&a[0]));
	const std::string name(basename(&b[0]));
	char* cwd = getcwd(NULL, 0);
	if (cwd == NULL || chdir(dir.c_str()) != 0) {
		printf("%s: cannot open %s\n", program, file);
		free(cwd);
		return false;
	}
	bool ok = false;
	try {
		config.readFile(name.c_str());
		ok = true;
	} catch (const libconfig::FileIOException&) {
		printf("%s: cannot read %s\n", program, file);
	} catch (const libconfig::ParseException& e) {
		printf("%s: %s line %d: %s\n", program, file, e.getLine(), e.getError());
	}
	if (chdir(cwd) != 0) {
		printf("%s: cannot return to %s\n", program, cwd);
		ok = false;
	}
	free(cwd);
	return ok;
}

/* The file ProductManager will read: BARRETT_CONFIG_FILE, else
 * ~/.barrett/default.conf. */
inline const char* configFile() {
	const char* file = std::getenv("BARRETT_CONFIG_FILE");
	if (file == NULL || file[0] == '\0') {
		file = barrett::ProductManager::DEFAULT_CONFIG_FILE.c_str();
	}
	return file;
}

/* Load that file and check the wam8g flags. Prints what it finds. */
inline bool loadAndCheck(libconfig::Config& config, const char* program) {
	const char* file = configFile();
	if (!readConfig(file, config, program)) {
		return false;
	}
	printf("Configuration: %s\n", file);
	return configReady(config, program);
}

}  // namespace wam8g_config
