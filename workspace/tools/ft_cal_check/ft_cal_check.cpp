// How well the poses of `follower --ft-cal` (src/lib/ft_cal_poses.h) spread gravity
// over the tool frame, for a follower configuration; no robot.
//   ft_cal_check <follower.conf> [group, default wam7w]
// Build against the follower's library (in the container):
//   g++ -std=c++11 ft_cal_check.cpp -I<package>/src/lib -I/opt/barrett/include -I/usr/include/eigen3 \
//       -L/opt/barrett/lib -lbarrett -lconfig -lgsl -lgslcblas -Wl,-rpath,/opt/barrett/lib -o ft_cal_check
#include <cmath>
#include <cstdio>
#include <string>
#include <libconfig.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <barrett/cdlbt/kinematics.h>
#include "ft_cal_poses.h"
#include "wam8g_teleop.h"

int main(int argc, char** argv) {
	if (argc < 2) {
		std::printf("usage: %s <follower.conf> [group]\n", argv[0]);
		return 2;
	}
	const std::string group = argc > 2 ? argv[2] : "wam7w";
	config_t cfg;
	config_init(&cfg);
	if (config_read_file(&cfg, argv[1]) != CONFIG_TRUE) {
		std::printf("%s: line %d: %s\n", argv[1], config_error_line(&cfg), config_error_text(&cfg));
		return 1;
	}
	config_setting_t* ks = config_lookup(&cfg, (group + ".kinematics").c_str());
	struct bt_kinematics* kin = NULL;
	if (ks == NULL || bt_kinematics_create(&kin, ks, 7)) {
		std::printf("no kinematics for %s in %s\n", group.c_str(), argv[1]);
		return 1;
	}
	const double lo[7] = {-2.6, -2.0, -2.8, -0.9, -4.76, -1.57, -3.0}, hi[7] = {2.6, 2.0, 2.8, 3.1, 1.24, 1.57, 3.0};
	gsl_vector* q = gsl_vector_alloc(7);
	gsl_vector* v = gsl_vector_calloc(7);
	gsl_matrix* G = gsl_matrix_alloc(ft_cal::POSES, 3);
	double mean[3] = {0.0, 0.0, 0.0};
	bool inside = true;
	std::printf("pose   J5     J6     J7   | gravity's direction in the tool frame\n");
	for (int k = 0; k < ft_cal::POSES; ++k) {
		for (int j = 0; j < 4; ++j) gsl_vector_set(q, j, wam8g_teleop::FOLLOWER_SYNC_POSE[j]);
		for (int j = 0; j < 3; ++j) gsl_vector_set(q, 4 + j, ft_cal::WRIST[k][j]);
		for (int j = 0; j < 7; ++j) {
			const double x = gsl_vector_get(q, j);
			if (x < lo[j] + 0.3 || x > hi[j] - 0.3) inside = false;  // 0.3 rad from every limit
		}
		bt_kinematics_eval(kin, q, v);
		double g[3];
		for (int i = 0; i < 3; ++i) {  // world -z in the tool's axes: R^T (0, 0, -1)
			g[i] = -gsl_matrix_get(kin->tool->rot_to_world, 2, i);
			gsl_matrix_set(G, k, i, g[i]);
			mean[i] += g[i] / ft_cal::POSES;
		}
		std::printf("%3d  %6.3f %6.3f %6.3f | %6.2f %6.2f %6.2f\n", k + 1, ft_cal::WRIST[k][0], ft_cal::WRIST[k][1],
		            ft_cal::WRIST[k][2], g[0], g[1], g[2]);
	}
	for (int k = 0; k < ft_cal::POSES; ++k)
		for (int i = 0; i < 3; ++i) gsl_matrix_set(G, k, i, gsl_matrix_get(G, k, i) - mean[i]);
	gsl_matrix* V = gsl_matrix_alloc(3, 3);
	gsl_vector* S = gsl_vector_alloc(3);
	gsl_vector* work = gsl_vector_alloc(3);
	gsl_linalg_SV_decomp(G, V, S, work);
	const double n = std::sqrt((double) ft_cal::POSES);
	std::printf("spread of gravity over the tool frame: %.2f, %.2f, %.2f (0.58 each is ideal; the fit needs 0.15)\n",
	            gsl_vector_get(S, 0) / n, gsl_vector_get(S, 1) / n, gsl_vector_get(S, 2) / n);
	std::printf("every joint at least 0.3 rad inside its range: %s\n", inside ? "yes" : "NO");
	return (gsl_vector_get(S, 2) / n >= 0.3 && inside) ? 0 : 1;
}
