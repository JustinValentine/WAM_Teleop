// The library's own gravity model, evaluated without the robot: reads joint
// positions (8 per line) on stdin and prints the 8 gravity-compensation torques
// the library would command there, from the configuration given as argv[1]
// (run from the configuration's directory so its @include lines resolve).
// Optional argv[2]: a file with a "mus" setting to use instead of the configured one.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <libconfig.h++>
#include <gsl/gsl_vector.h>
#include <barrett/detail/libconfig_compat.h>
#include <barrett/cdlbt/kinematics.h>
#include <barrett/cdlbt/calgrav.h>
int main(int argc, char** argv) {
	if (argc < 2) { std::fprintf(stderr, "usage: gravtool default.conf [gravitycal.conf] < positions\n"); return 2; }
	libconfig::Config cfg, alt;
	try { cfg.readFile(argv[1]); if (argc > 2) alt.readFile(argv[2]); }
	catch (const libconfig::ParseException& e) { std::fprintf(stderr, "parse error %s:%d %s\n", e.getFile(), e.getLine(), e.getError()); return 1; }
	libconfig::Setting& s = cfg.lookup("wam8g");
	struct bt_kinematics* kin = NULL; struct bt_calgrav* grav = NULL;
	if (bt_kinematics_create(&kin, barrett::detail::getCSetting(s["kinematics"]), 8)) { std::fprintf(stderr, "kinematics failed\n"); return 1; }
	libconfig::Setting& g = (argc > 2) ? alt.lookup("gravity_compensation") : s["gravity_compensation"];
	if (bt_calgrav_create(&grav, barrett::detail::getCSetting(g), 8)) { std::fprintf(stderr, "calgrav failed\n"); return 1; }
	gsl_vector* q = gsl_vector_alloc(8); gsl_vector* v = gsl_vector_calloc(8); gsl_vector* t = gsl_vector_alloc(8);
	std::string line;
	while (std::getline(std::cin, line)) {
		std::istringstream is(line); bool ok = true;
		for (int j = 0; j < 8; ++j) { double x; if (!(is >> x)) { ok = false; break; } gsl_vector_set(q, j, x); }
		if (!ok) continue;
		bt_kinematics_eval(kin, q, v);
		bt_calgrav_eval(grav, kin, t);
		for (int j = 0; j < 8; ++j) std::printf("%s%.6f", j ? " " : "", gsl_vector_get(t, j));
		std::printf("\n");
	}
	return 0;
}
