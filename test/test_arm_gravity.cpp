/* lib/arm_gravity.h (libbarrett's calgrav, read from an arm's configuration)
 * against the identification's Python version (ident/cable_ident/config.py,
 * test/data/arm_gravity_*.txt), on each arm's own configuration file.
 * Usage: arm_gravity_tests <wam8g.conf> <wam7w.conf> (the package's test
 * passes config/wam8g/wam8g.conf and config/follower-zeus-hand/wam7w.conf). */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "arm_gravity.h"
#include "wam8g_config.h"

#ifndef TEST_DATA_DIR
#define TEST_DATA_DIR "test/data"
#endif

static int check(const char* file, const char* group, const char* name, int dof) {
	libconfig::Config cfg;
	if (!wam8g_config::readConfig(file, cfg, "arm_gravity_tests")) {
		return 1;
	}
	arm_gravity::Model g;
	if (!g.load(cfg.lookup(group), "arm_gravity_tests") || g.getDof() != dof) {
		printf("FAIL %s: no %d-joint gravity model in %s\n", name, dof, file);
		return 1;
	}
	std::ifstream in((std::string(TEST_DATA_DIR) + "/arm_gravity_" + name + ".txt").c_str());
	std::string line;
	double worst = 0.0;
	int rows = 0;
	while (std::getline(in, line)) {
		if (line.empty() || line[0] == '#') {
			continue;
		}
		std::istringstream s(line);
		double q[8], tau[8], dg[4], t[8];
		for (int j = 0; j < dof; ++j) s >> q[j];
		for (int j = 0; j < dof; ++j) s >> tau[j];
		for (int j = 0; j < 4; ++j) s >> dg[j];
		g.torque(q, t);
		for (int j = 0; j < dof; ++j) worst = std::max(worst, std::fabs(t[j] - tau[j]));
		const double ref[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		const Eigen::Matrix<double, 4, 1> c = g.wristCorrection(q, 4, ref);
		for (int j = 0; j < 4; ++j) worst = std::max(worst, std::fabs(c[j] - dg[j]));
		++rows;
	}
	printf("%s: %d poses, largest difference from the identification's gravity %.2e Nm\n", name, rows, worst);
	return (rows < 100 || worst > 1e-9) ? 1 : 0;
}

int main(int argc, char** argv) {
	if (argc != 3) {
		printf("usage: %s <wam8g.conf> <wam7w.conf>\n", argv[0]);
		return 2;
	}
	int f = check(argv[1], "wam8g", "leader", 8) + check(argv[2], "wam7w", "follower", 7);
	printf(f ? "%d FAILED\n" : "all passed\n", f);
	return f ? 1 : 0;
}
